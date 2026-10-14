#!/usr/bin/env python3
"""
Estrae le metriche di riferimento da un tdma_security_log.csv.

Uso (singola simulazione):
    python3 tools/metrics.py tdma_security_log.csv --attack-time 200 --targets 0 \
        --n-base 8 --out metrics.json

Uso (aggregazione di piu' seed):
    python3 tools/metrics.py --aggregate runs/base/seed*/metrics.json --out summary.json

NOTA: la colonna `alarm` del CSV e' l'ALLARME COLLETTIVO (voti >= soglia),
non il sospetto individuale del singolo osservatore.
"""
import argparse
import json
import math
import sys

import pandas as pd


def _pct(num, den):
    return 100.0 * num / den if den else float("nan")


def compute(log_path, attack_time, targets, n_base):
    d = pd.read_csv(log_path)
    m = {"righe_log": int(len(d))}

    pre = d[d.time < attack_time]
    post = d[d.time >= attack_time]

    # 1) Falsi allarmi collettivi prima dell'attacco (tutti i mittenti sono onesti)
    m["fa_collettivi_pre_attacco_pct"] = _pct(int(pre.alarm.sum()), len(pre))
    m["fa_pre_per_mittente"] = {str(k): int(v) for k, v in
                               pre[pre.alarm == 1].groupby("sender_id").size().items()}

    # 2) Rilevamento sui bersagli dell'attacco
    det = {}
    for t in targets:
        obs = post[post.sender_id == t]
        seen = sorted(int(o) for o in obs.observer_id.unique())
        first = obs[obs.alarm == 1].groupby("observer_id").time.min() - attack_time
        lat = sorted(float(x) for x in first.values)
        det[str(t)] = {
            "osservatori_che_lo_vedono": len(seen),
            "osservatori_che_rilevano": len(lat),
            "latenza_min_s": lat[0] if lat else None,
            "latenza_mediana_s": float(pd.Series(lat).median()) if lat else None,
            "latenza_max_s": lat[-1] if lat else None,
            # frazione di osservazioni dopo l'attacco con allarme attivo
            "tasso_allarme_post_pct": _pct(int(obs.alarm.sum()), len(obs)),
        }
    m["rilevamento"] = det

    # 3) Falsi allarmi collettivi sui nodi ONESTI dopo l'attacco
    honest_post = post[~post.sender_id.isin(targets)]
    m["fa_collettivi_onesti_post_attacco_pct"] = _pct(int(honest_post.alarm.sum()),
                                                     len(honest_post))
    m["fa_post_per_mittente"] = {str(k): int(v) for k, v in
                                honest_post[honest_post.alarm == 1]
                                .groupby("sender_id").size().items()}

    # 3b) Metriche INDIVIDUALI (colonne presenti dalla Fase 1.1 in poi)
    if "my_vote" in d.columns:
        m["fa_individuali_pre_attacco_pct"] = _pct(int(pre.my_vote.sum()), len(pre))
        m["sospetti_istantanei_pre_attacco_pct"] = _pct(int(pre.suspicious_now.sum()), len(pre))
        m["fa_individuali_onesti_post_attacco_pct"] = _pct(int(honest_post.my_vote.sum()),
                                                          len(honest_post))
        for t in targets:
            obs = post[post.sender_id == t]
            first = obs[obs.my_vote == 1].groupby("observer_id").time.min() - attack_time
            lat = sorted(float(x) for x in first.values)
            det[str(t)]["latenza_individuale_mediana_s"] = (
                float(pd.Series(lat).median()) if lat else None)
            det[str(t)]["tasso_voto_individuale_post_pct"] = _pct(int(obs.my_vote.sum()), len(obs))

    # 3c) Solo tra droni ONESTI: osservatore e drone osservato non sono bersagli dell'attacco.
    #     (le metriche sopra includono anche le opinioni dell'attaccante sugli altri)
    hh = post[~post.sender_id.isin(targets) & ~post.observer_id.isin(targets)]
    m["fa_collettivi_onesti_da_onesti_post_pct"] = _pct(int(hh.alarm.sum()), len(hh))
    if "my_vote" in d.columns:
        m["fa_individuali_onesti_da_onesti_post_pct"] = _pct(int(hh.my_vote.sum()), len(hh))
    m["errore_ekf_onesti_post_mediana_m"] = float(hh.estimation_error.median()) if len(hh) else None
    m["errore_ekf_onesti_post_p95_m"] = float(hh.estimation_error.quantile(.95)) if len(hh) else None

    # 4) Accuratezza della multilaterazione (EKF) prima dell'attacco
    base = pre[pre.sender_id < n_base].estimation_error
    guest = pre[pre.sender_id >= n_base].estimation_error
    m["errore_ekf_base_mediana_m"] = float(base.median()) if len(base) else None
    m["errore_ekf_base_p95_m"] = float(base.quantile(.95)) if len(base) else None
    m["errore_ekf_ospiti_mediana_m"] = float(guest.median()) if len(guest) else None
    m["errore_ekf_ospiti_p95_m"] = float(guest.quantile(.95)) if len(guest) else None
    # p99: le divergenze brevi di un solo drone (passo 6.2) restavano sotto il p95
    m["errore_ekf_base_p99_m"] = float(base.quantile(.99)) if len(base) else None
    m["errore_ekf_ospiti_p99_m"] = float(guest.quantile(.99)) if len(guest) else None

    # 5) Dal passo 6.2. Con il ricevitore GNSS lo sciame si localizza nel riferimento del GPS, spostato
    #    dell'errore comune: l'errore che conta e' quello RELATIVO a quel riferimento. L'errore comune si
    #    stima per ogni decimo di secondo come mediana, sui mittenti, di (GPS dichiarato - posizione vera).
    #    Si escludono i primi 20 s (inizializzazione dei filtri)
    w = pre[pre.time > 20].copy()
    w["tb"] = (w.time * 10).round()
    for a in "xyz":
        w["c" + a] = w["claim_" + a] - w["true_" + a]
    com = w.groupby(["tb", "sender_id"])[["cx", "cy", "cz"]].first().groupby("tb").median()
    w = w.join(com, on="tb", rsuffix="_com")
    rel = sum((w["est_" + a] - w["true_" + a] - w["c" + a + "_com"]) ** 2 for a in "xyz") ** 0.5
    wb = w.sender_id < n_base
    for name, sel in (("base", wb), ("ospiti", ~wb)):
        if sel.any():
            m[f"errore_rel_{name}_mediana_m"] = float(rel[sel].median())
            m[f"errore_rel_{name}_p95_m"] = float(rel[sel].quantile(.95))
    #    Calibrazione del test chi-quadro (3 g.d.l.): media attesa 3, frazione oltre 16.27 attesa 0.1%
    if "gps_chi2" in w.columns and len(w):
        m["chi2_media_pre"] = float(w.gps_chi2.mean())
        m["chi2_oltre_999_pre_pct"] = _pct(int((w.gps_chi2 > 16.27).sum()), len(w))

    # 6) Dalla Fase 8: cattura dello sciame (spoofer esterno). Lo stato e' dell'osservatore: una riga per
    #    osservatore e decimo di secondo. Senza spoofer deve restare 0 anche con un drone che mente (nodo 0)
    if "swarm_capture" in d.columns:
        st = d.assign(tb=(d.time * 10).round()).groupby(["observer_id", "tb"]).agg(
            time=("time", "first"), cap=("swarm_capture", "max"), ok=("pairs_ok_frac", "first")).reset_index()
        spre = st[(st.time > 20) & (st.time < attack_time)]
        spost = st[st.time >= attack_time + 5]
        m["cattura_pre_attacco_pct"] = _pct(int(spre.cap.sum()), len(spre))
        m["cattura_post_attacco_pct"] = _pct(int(spost.cap.sum()), len(spost))
        m["coppie_coerenti_pre_mediana"] = float(spre.ok.median()) if len(spre) else None
        m["coppie_coerenti_post_mediana"] = float(spost.ok.median()) if len(spost) else None
        first = st[(st.time >= attack_time) & (st.cap == 1)].groupby("observer_id").time.min() - attack_time
        n_obs = st[st.time >= attack_time].observer_id.nunique()
        m["cattura_osservatori_che_rilevano"] = int(len(first))
        m["cattura_osservatori_attivi"] = int(n_obs)
        m["cattura_latenza_mediana_s"] = float(first.median()) if len(first) else None
    return m


def _flat(m, prefix=""):
    """Appiattisce il dizionario in chiave -> valore numerico."""
    out = {}
    for k, v in m.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(_flat(v, key + "."))
        elif isinstance(v, (int, float)) and v is not None:
            out[key] = float(v)
    return out


def aggregate(paths):
    rows = [_flat(json.load(open(p))) for p in paths]
    keys = sorted(set().union(*rows))
    agg = {"n_seed": len(rows)}
    for k in keys:
        # i conteggi per mittente dipendono dal seed: non ha senso mediarli
        if ".fa_pre_per_mittente." in f".{k}" or ".fa_post_per_mittente." in f".{k}":
            continue
        vals = [r[k] for r in rows if k in r and not math.isnan(r[k])]
        if not vals:
            continue
        s = pd.Series(vals)
        agg[k] = {"media": float(s.mean()),
                  "std": float(s.std(ddof=1)) if len(s) > 1 else 0.0,
                  "min": float(s.min()), "max": float(s.max()), "n": len(s)}
    return agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?")
    ap.add_argument("--attack-time", type=float, default=200.0)
    ap.add_argument("--targets", default="0", help="ID bersaglio separati da virgola")
    ap.add_argument("--n-base", type=int, default=8)
    ap.add_argument("--aggregate", nargs="+", metavar="JSON")
    ap.add_argument("--out")
    a = ap.parse_args()

    if a.aggregate:
        res = aggregate(a.aggregate)
    elif a.log:
        targets = [int(x) for x in a.targets.split(",") if x.strip()]
        res = compute(a.log, a.attack_time, targets, a.n_base)
    else:
        ap.error("serve un file di log oppure --aggregate")

    txt = json.dumps(res, indent=2, ensure_ascii=False)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    sys.exit(main())
