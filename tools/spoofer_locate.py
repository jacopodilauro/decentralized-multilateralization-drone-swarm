#!/usr/bin/env python3
"""Localizzazione dello spoofer con la geometria STIMATA dallo sciame (Fase 9, primo passo, analisi dei log).

Uso: python3 tools/spoofer_locate.py runs/<etichetta>/seed1 [--observer 1] [--window 20] [--jitter-ns 0]
                                    [--png figura.png]

Per un drone osservatore, sotto consenso di cattura (8.2b): ogni PPS del drone j (1 Hz) da' una misura
r_j = |p_j - s| + c*beta, con p_j = STIMA dell'osservatore (localizzazione relativa UWB), non la verita'.
Minimi quadrati su (s, c*beta). Tre varianti:
  ideale        posizioni vere, misure accumulate dall'inizio della cattura;
  stimata       posizioni stimate COSI' COME SONO (nessun allineamento alla verita'), misure accumulate
                dall'inizio: e' il risultato reale del sistema. Senza 8.2c il riferimento stimato ruota e scivola
                e il risultato e' inutilizzabile; con 8.2c (sensori di bordo, default) resta agganciato al mondo;
  8.2c ideale   posizioni stimate riportate nel mondo OGNI SECONDO con la miglior rototraslazione (= riferimento
                ancorato perfettamente, errori di forma intatti), misure accumulate dall'inizio.
Diagnosi (seed 1): il riferimento stimato ruota di ~3-5 gradi/s (picchi 60-90 gradi/s): e' la causa principale.
IPOTESI IDEALI dichiarate: sincronizzazione (offset UWB tra orologi perfetti), orientamento nel mondo.
L'osservatore non stima se stesso nel CSV: si usano gli altri droni (19 con gli ospiti).
"""
import argparse, os, sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spoofer_view import antenna_from_stdout, solve_toa  # noqa: E402


def kabsch(E, T):
    ce, ct = E.mean(axis=0), T.mean(axis=0)
    U, _, Vt = np.linalg.svd((E - ce).T @ (T - ct))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, ce, ct


def errors(s_world, center, s_true):
    v, vt = s_world - center, s_true - center
    daz = (np.degrees(np.arctan2(v[1], v[0]) - np.arctan2(vt[1], vt[0])) + 180) % 360 - 180
    del_ = np.degrees(np.arctan2(v[2], np.linalg.norm(v[:2])) - np.arctan2(vt[2], np.linalg.norm(vt[:2])))
    return daz, del_, np.linalg.norm(v) - np.linalg.norm(vt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rundir")
    ap.add_argument("--observer", type=int, default=1)
    ap.add_argument("--window", type=float, default=20.0)
    ap.add_argument("--jitter-ns", type=float, default=0.0)
    ap.add_argument("--png", default="")
    a = ap.parse_args()

    s_true, t0 = antenna_from_stdout(os.path.join(a.rundir, "stdout.txt"))
    g = pd.read_csv(os.path.join(a.rundir, "gnss_log.csv"))
    d = pd.read_csv(os.path.join(a.rundir, "tdma_security_log.csv"),
                    usecols=["time", "observer_id", "sender_id", "est_x", "est_y", "est_z",
                             "true_x", "true_y", "true_z", "capture_consensus"])
    o = d[d.observer_id == a.observer].sort_values("time")
    cons = o[o.capture_consensus == 1]
    if cons.empty:
        sys.exit("l'osservatore non arriva mai al consenso di cattura: run con --spoof=1?")
    t_rel = cons.time.min()

    # un PPS per drone e per secondo, solo sotto consenso, solo droni che l'osservatore stima
    p = g[(g.spoofed == 1) & (g.time >= t_rel) & (g.node != a.observer)].copy()
    p = p.assign(sec=np.floor(p.time)).groupby(["node", "sec"]).first().reset_index().sort_values("time")
    rng = np.random.default_rng(1)
    p["r"] = p.pps_err_m + 0.299792458 * a.jitter_ns * rng.standard_normal(len(p))
    rows = []
    for j, pj in p.groupby("node"):
        oj = o[(o.sender_id == j) & (o.capture_consensus == 1)][["time", "est_x", "est_y", "est_z"]]
        if oj.empty:
            continue
        m = pd.merge_asof(pj.sort_values("time"), oj, on="time", direction="nearest", tolerance=0.2)
        rows.append(m.dropna(subset=["est_x"]))
    m = pd.concat(rows).sort_values("time")
    E = m[["est_x", "est_y", "est_z"]].values
    T = m[["true_x", "true_y", "true_z"]].values
    r = m.r.values
    t = m.time.values

    print(f"Spoofer vero {np.round(s_true, 1)}; osservatore {a.observer}, consenso a t = {t_rel:.2f} s; "
          f"{m.node.nunique()} droni, {len(m)} PPS")
    print("Ipotesi ideali: sincronizzazione UWB, orientamento nel mondo (allineamento alla verita').\n")
    print(f"{'dopo':>6s} | {'geometria ideale':^27s} | {'stimata (risultato reale)':^27s} | "
          f"{'stimata, 8.2c ideale':^27s}")
    print(f"{'':6s} | {'azimut':>8s} {'elev.':>7s} {'dist.':>9s} | {'azimut':>8s} {'elev.':>7s} {'dist.':>9s} | "
          f"{'azimut':>8s} {'elev.':>7s} {'dist.':>9s}")
    # 8.2c ideale: ogni secondo le stime vengono riportate nel mondo (rotazione e deriva tolte, forma intatta)
    Ew = np.full_like(E, np.nan)
    for s_ in np.unique(np.floor(t)):
        k = np.floor(t) == s_
        if k.sum() >= 4:
            R, ce, ct = kabsch(E[k], T[k])
            Ew[k] = (R @ (E[k] - ce).T).T + ct
    okw = ~np.isnan(Ew[:, 0])
    out = []
    for T_end in np.arange(5, t.max() - t_rel + 1e-6, 5.0):
        te = t_rel + T_end
        cum = t <= te
        win = cum & (t > te - a.window)
        if cum.sum() < 20 or win.sum() < 20 or (cum & okw).sum() < 20:
            continue
        res = [T_end]
        # ideale cumulativa
        x = solve_toa(T[cum], r[cum]); c = T[win].mean(axis=0)
        res += list(errors(x[:3], c, s_true))
        # stimata cosi' com'e': nessun allineamento
        x = solve_toa(E[cum], r[cum])
        res += list(errors(x[:3], c, s_true))
        # 8.2c ideale: riferimento corretto ogni secondo, cumulativa
        kk = cum & okw
        x = solve_toa(Ew[kk], r[kk])
        res += list(errors(x[:3], c, s_true))
        out.append(res)
        print(f"{T_end:5.0f}s | {res[1]:+7.2f}° {res[2]:+6.2f}° {res[3]:+8.0f}m | {res[4]:+7.2f}° {res[5]:+6.2f}° "
              f"{res[6]:+8.0f}m | {res[7]:+7.2f}° {res[8]:+6.2f}° {res[9]:+8.0f}m")
    out = np.array(out)
    if len(out):
        late = out[:, 0] >= 20
        for name, k in (("geometria ideale", 1), ("stimata, reale", 4), ("stimata, 8.2c ideale", 7)):
            print(f"  dopo 20 s, {name:19s}: |azimut| mediano {np.median(np.abs(out[late, k])):5.2f}°, "
                  f"|distanza| mediana {np.median(np.abs(out[late, k + 2])):5.0f} m")

    if a.png and len(out):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for k, lab, st in ((1, "geometria ideale (posizioni vere)", "k--"),
                           (4, "geometria stimata dallo sciame (risultato reale)", "C3-"),
                           (7, "geometria stimata dallo sciame, con 8.2c ideale", "C0-")):
            ax[0].plot(out[:, 0], np.abs(out[:, k]), st, label=lab)
            ax[1].plot(out[:, 0], np.abs(out[:, k + 2]), st, label=lab)
        ax[0].set_ylabel("errore di azimut [gradi]"); ax[1].set_ylabel("errore di distanza [m]")
        ax[0].set_yscale("log"); ax[1].set_yscale("log")
        for x_ in ax:
            x_.set_xlabel("secondi dall'inizio della localizzazione relativa"); x_.grid(alpha=.3)
        ax[0].legend(fontsize=8)
        fig.suptitle(f"Localizzazione dello spoofer a {np.linalg.norm((s_true - T.mean(axis=0))[:2]):.0f} m "
                     f"dai PPS dello sciame catturato (osservatore {a.observer})", fontsize=10)
        fig.tight_layout(); fig.savefig(a.png, dpi=150)
        print(f"\nFigura: {a.png}")


if __name__ == "__main__":
    main()
