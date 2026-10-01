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

    # 4) Accuratezza della multilaterazione (EKF) prima dell'attacco
    base = pre[pre.sender_id < n_base].estimation_error
    guest = pre[pre.sender_id >= n_base].estimation_error
    m["errore_ekf_base_mediana_m"] = float(base.median()) if len(base) else None
    m["errore_ekf_base_p95_m"] = float(base.quantile(.95)) if len(base) else None
    m["errore_ekf_ospiti_mediana_m"] = float(guest.median()) if len(guest) else None
    m["errore_ekf_ospiti_p95_m"] = float(guest.quantile(.95)) if len(guest) else None
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
