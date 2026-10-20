#!/usr/bin/env python3
"""Diagnosi per osservatore della geometria stimata sotto cattura e della direzione dello spoofer.

Uso: python3 tools/observer_diag.py runs/<etichetta>/seed<N> [--per-second OSS]

Per ogni osservatore base (ID < 8), sotto consenso di cattura (da 5 s dopo l'attivazione):
  az_reale     errore di azimut dello spoofer (geometria stimata + PPS reali, come aperture_sweep.py)
  forma        residuo RMS dopo la miglior ROTOTRASLAZIONE (Kabsch, det = +1), mediana per secondo
  forma_rifl   residuo RMS ammettendo anche una RIFLESSIONE (det = -1): molto minore di 'forma' = immagine specchiata
  specchio     frazione dei secondi in cui la riflessione spiega meglio di 0,5x la rotazione
  rot_z        rotazione attorno alla verticale della miglior rototraslazione (mediana) [gradi]
  rx           pacchetti ricevuti dagli altri droni base / attesi (righe CSV, 10 al secondo per mittente)
  dist         distanza vera media dagli altri droni base [m]
  pps_out      differenze di PPS con |misurata - ideale| > 20 m (diagnostica con la verita' del simulatore: la
               sincronizzazione reale ha errori di pochi m; valori di km indicano una relazione tra orologi sbagliata)
Con --per-second OSS stampa l'andamento nel tempo per quell'osservatore.
"""
import argparse, os, sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spoofer_view import antenna_from_stdout  # noqa: E402
from spoofer_locate import errors, solve_tdoa_epochs  # noqa: E402
from aperture_sweep import observer_table  # noqa: E402

N_BASE = 8


def kabsch(E, T, allow_reflection=False):
    """Miglior trasformazione rigida E -> T. Restituisce residuo RMS, matrice e determinante."""
    ce, ct = E.mean(axis=0), T.mean(axis=0)
    A, B = E - ce, T - ct
    U, _, Vt = np.linalg.svd(A.T @ B)
    d = np.sign(np.linalg.det(U @ Vt))
    D = np.diag([1.0, 1.0, d if not allow_reflection else 1.0])
    R = U @ D @ Vt                     # applicata come A @ R
    res = A @ R - B
    return float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))), R


def per_second(o):
    """Per ogni secondo: stime dell'osservatore di TUTTI gli altri droni (ultima riga del secondo)."""
    o = o.assign(sec=np.floor(o.time).astype(int))
    last = o.sort_values("time").groupby(["sec", "sender_id"]).last().reset_index()
    out = []
    for sec, g in last.groupby("sec"):
        if len(g) < 4:
            continue
        E = g[["est_x", "est_y", "est_z"]].values
        T = g[["true_x", "true_y", "true_z"]].values
        r, R = kabsch(E, T)
        rr, _ = kabsch(E, T, allow_reflection=True)
        # angolo attorno alla verticale: proiezione della rotazione sul piano orizzontale
        yaw = np.degrees(np.arctan2(R[0, 1], R[0, 0]))
        out.append((sec, len(g), r, rr, yaw))
    return pd.DataFrame(out, columns=["sec", "n", "forma", "forma_rifl", "rot_z"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--per-second", type=int, default=-1)
    a = ap.parse_args()
    s_true, t0 = antenna_from_stdout(os.path.join(a.run, "stdout.txt"))
    d = pd.read_csv(os.path.join(a.run, "tdma_security_log.csv"),
                    usecols=["time", "observer_id", "sender_id", "est_x", "est_y", "est_z",
                             "true_x", "true_y", "true_z", "capture_consensus"])
    pl = pd.read_csv(os.path.join(a.run, "pps_log.csv"))
    cap = d[(d.time >= t0 + 5) & (d.capture_consensus == 1)]
    dur = cap.time.max() - cap.time.min()
    rows = []
    for obs in range(N_BASE):
        o = cap[cap.observer_id == obs]
        if o.empty:
            continue
        ps = per_second(o)
        m = observer_table(pl, d, obs)
        az = np.nan
        if m is not None and m.sec.nunique() >= 10:
            E = m[["est_x", "est_y", "est_z"]].values
            T = m[["true_x", "true_y", "true_z"]].values
            ep = m.sec.values
            c = T[ep > ep.max() - 5].mean(axis=0)
            az, _, _ = errors(solve_tdoa_epochs(E, m.d_meas_m.values, ep), c, s_true)
        ob = o[o.sender_id < N_BASE]
        rx = len(ob) / (10.0 * dur * (N_BASE - 1))
        mine = d[(d.sender_id == obs)][["time", "true_x", "true_y", "true_z"]]
        oth = ob[["time", "sender_id", "true_x", "true_y", "true_z"]]
        mm = pd.merge_asof(oth.sort_values("time"), mine.rename(columns=lambda c: c + "_o" if c != "time" else c)
                           .sort_values("time"), on="time", direction="nearest", tolerance=0.1).dropna()
        dist = np.mean(np.linalg.norm(mm[["true_x", "true_y", "true_z"]].values
                                      - mm[["true_x_o", "true_y_o", "true_z_o"]].values, axis=1)) if len(mm) else np.nan
        po = pl[pl.observer == obs]
        pps_out = int(((po.d_meas_m - po.d_ideal_m).abs() > 20.0).sum())
        rows.append(dict(oss=obs, az_reale=az, pps_out=pps_out, forma=ps.forma.median(), forma_p95=ps.forma.quantile(0.95),
                         forma_rifl=ps.forma_rifl.median(), specchio=float(np.mean(ps.forma_rifl < 0.5 * ps.forma)),
                         rot_z=ps.rot_z.median(), rx=rx, dist=dist))
        if obs == a.per_second:
            pd.set_option("display.width", 200)
            print(ps.iloc[::5].round(2).to_string(index=False))
    pd.set_option("display.width", 200)
    print(pd.DataFrame(rows).round(3).to_string(index=False))


if __name__ == "__main__":
    main()
