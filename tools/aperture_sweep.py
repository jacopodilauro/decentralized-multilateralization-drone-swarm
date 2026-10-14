#!/usr/bin/env python3
"""Strada A: distanza (e direzione) dello spoofer in funzione dell'apertura dello sciame.

Uso:
  python3 tools/aperture_sweep.py --one runs/<etichetta>/seed<N> --out risultati.csv   # una run -> una riga
  python3 tools/aperture_sweep.py --plot risultati.csv --png figura.png               # tabella + figura

Per ogni run (con --spoof=1) calcola:
  - apertura orizzontale dello sciame catturato (mediana per secondo, posizioni vere; solo informativa: gli
    ospiti escono a ~167 s e la mediana salta tra i due regimi, per questo le figure usano la scala S);
  - segnale di curvatura: differenze di distanza vere meno quelle di un'onda piana dalla stessa direzione
    (RMS tra droni, mediana per secondo): e' cio' che porta l'informazione sulla distanza;
  - errore di forma della geometria stimata dallo sciame sotto cattura (Kabsch, come metrics.py);
  - per ogni osservatore base (ID < 8): stima dello spoofer con tutte le differenze di PPS dal consenso alla
    fine, in due varianti: REALE (geometria stimata + PPS e orologi UWB reali) e IDEALE (posizioni vere +
    PPS ideali). Errori di direzione (azimut) e di distanza; "determinata" se |errore di distanza| < 50% della
    distanza vera; "fusione" = mediana delle direzioni dei droni base (stima dello sciame).
Regola fisica attesa: la distanza e' osservabile quando la curvatura (~ (a/2)^2 / 2R) supera l'errore di forma.
"""
import argparse, os, re, sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spoofer_view import antenna_from_stdout  # noqa: E402
from spoofer_locate import errors, solve_tdoa_epochs  # noqa: E402
from metrics import _shape_stats  # noqa: E402

N_BASE = 8


def scale_from_stdout(path):
    with open(path) as f:
        for line in f:
            m = re.search(r"Formazione scalata: S=(\S+)", line)
            if m:
                return float(m.group(1))
    return 1.0


def curvature_and_aperture(g, s):
    g = g[g.spoofed == 1]
    g = g.assign(sec=np.floor(g.time)).groupby(["node", "sec"]).first().reset_index()
    cur, ap = [], []
    for _, gg in g.groupby("sec"):
        X = gg[["true_x", "true_y", "true_z"]].values
        if len(X) < 4:
            continue
        c = X.mean(axis=0)
        u = (s - c) / np.linalg.norm(s - c)
        rho = np.linalg.norm(X - s, axis=1)
        plane = -(X - c) @ u
        r = (rho - rho.mean()) - (plane - plane.mean())
        cur.append(np.sqrt(np.mean(r ** 2)))
        ap.append(np.ptp(X[:, :2], axis=0).max())
    return float(np.median(cur)), float(np.median(ap))


def observer_table(pl, d, obs):
    p = pl[pl.observer == obs]
    o = d[(d.observer_id == obs) & (d.capture_consensus == 1)]
    rows = []
    for j, pj in p.groupby("sender"):
        oj = o[o.sender_id == j][["time", "est_x", "est_y", "est_z", "true_x", "true_y", "true_z"]]
        if oj.empty:
            continue
        oj = oj.rename(columns={"time": "tq"}).sort_values("tq")
        m = pd.merge_asof(pj.assign(tq=pj.sec.astype(float)).sort_values("tq"), oj, on="tq",
                          direction="nearest", tolerance=0.2)
        rows.append(m.dropna(subset=["est_x"]))
    if not rows:
        return None
    m = pd.concat(rows).sort_values("sec")
    return m[m.groupby("sec").sender.transform("size") >= 4]


def analyse_one(rundir):
    s_true, t0 = antenna_from_stdout(os.path.join(rundir, "stdout.txt"))
    S = scale_from_stdout(os.path.join(rundir, "stdout.txt"))
    g = pd.read_csv(os.path.join(rundir, "gnss_log.csv"))
    pl = pd.read_csv(os.path.join(rundir, "pps_log.csv"))
    d = pd.read_csv(os.path.join(rundir, "tdma_security_log.csv"),
                    usecols=["time", "observer_id", "sender_id", "est_x", "est_y", "est_z",
                             "true_x", "true_y", "true_z", "capture_consensus"])
    cur, ap = curvature_and_aperture(g, s_true)
    center = g[g.time < t0][["true_x", "true_y", "true_z"]].mean().values
    R = float(np.linalg.norm((s_true - center)[:2]))
    cap = d[(d.time >= t0 + 5) & (d.capture_consensus == 1)]
    sh = _shape_stats(cap)
    shape = float(np.median([x[0] for x in sh])) if sh else float("nan")
    res = {"run": rundir, "S": S, "R_m": R, "apertura_m": ap, "curvatura_m": cur, "forma_m": shape}
    per = {"reale": [], "ideale": []}
    for obs in range(N_BASE):
        m = observer_table(pl, d, obs)
        if m is None or m.sec.nunique() < 10:
            continue
        E = m[["est_x", "est_y", "est_z"]].values
        T = m[["true_x", "true_y", "true_z"]].values
        ep = m.sec.values
        c = T[ep > ep.max() - 5].mean(axis=0)
        for name, P, dd in (("reale", E, m.d_meas_m.values), ("ideale", T, m.d_ideal_m.values)):
            az, el, dr = errors(solve_tdoa_epochs(P, dd, ep), c, s_true)
            per[name].append((abs(az), abs(dr), az))
    for name, v in per.items():
        if v:
            a = np.array(v)
            res[f"{name}_osservatori"] = len(a)
            res[f"{name}_azimut_mediano_gradi"] = float(np.median(a[:, 0]))
            res[f"{name}_azimut_min_gradi"] = float(np.min(a[:, 0]))
            res[f"{name}_azimut_max_gradi"] = float(np.max(a[:, 0]))
            res[f"{name}_distanza_mediana_m"] = float(np.median(a[:, 1]))
            res[f"{name}_distanza_determinata_pct"] = float(100 * np.mean(a[:, 1] < 0.5 * R))
            # fusione: lo sciame prende la mediana delle direzioni dei droni base (errori con segno)
            res[f"{name}_azimut_fusione_gradi"] = float(abs(np.median(a[:, 2])))
    return res


def plot(csv, png):
    r = pd.read_csv(csv)
    cols = ["S", "R_m", "apertura_m", "curvatura_m", "forma_m", "reale_azimut_mediano_gradi", "reale_azimut_max_gradi",
            "reale_azimut_fusione_gradi",
            "reale_distanza_mediana_m", "reale_distanza_determinata_pct", "ideale_distanza_mediana_m"]
    r = r.assign(R_m=(r.R_m / 50).round() * 50)   # distanze nominali (il centro pre-cattura varia di pochi m)
    cols = [c for c in cols if c in r.columns]
    agg = r.groupby(["R_m", "S"])[cols[2:]].median().reset_index()
    agg["seed"] = r.groupby(["R_m", "S"]).size().values
    pd.set_option("display.width", 200)
    print(agg.round(2).to_string(index=False))
    if not png:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for i, (Rv, gg) in enumerate(agg.groupby("R_m")):
        c = f"C{i}"
        ax[0].plot(gg.S, 100 * gg.reale_distanza_mediana_m / Rv, "o-", color=c, label=f"reale, R = {Rv:.0f} m")
        ax[0].plot(gg.S, 100 * gg.ideale_distanza_mediana_m / Rv, "x--", color=c, alpha=.6,
                   label=f"ideale, R = {Rv:.0f} m")
        ax[1].plot(gg.S, gg.reale_azimut_mediano_gradi, "o-", color=c, label=f"R = {Rv:.0f} m")
        ax[2].plot(gg.S, gg.curvatura_m, "o-", color=c, label=f"curvatura, R = {Rv:.0f} m")
    ax[2].plot(agg.groupby("S").forma_m.median().index, agg.groupby("S").forma_m.median(), "ks--",
               label="errore di forma stimato")
    ax[0].set_yscale("log"); ax[0].axhline(50, color="grey", lw=.8, ls=":")
    ax[0].set_ylabel("errore di distanza mediano [% di R]")
    ax[1].set_ylabel("errore di direzione mediano [gradi]")
    ax[2].set_ylabel("[m]"); ax[2].set_yscale("log")
    for a in ax:
        a.set_xlabel("scala S (sfera base: diametro 20 S m; orbita ospiti: 30 S m)"); a.grid(alpha=.3); a.legend(fontsize=7)
    fig.suptitle("Strada A: la distanza dello spoofer diventa misurabile quando la curvatura supera l'errore di forma"
                 f" (seed per punto: {agg.seed.min()}-{agg.seed.max()})", fontsize=10)
    fig.tight_layout(); fig.savefig(png, dpi=150)
    print(f"Figura: {png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", help="cartella di una run (runs/<etichetta>/seed<N>)")
    ap.add_argument("--out", default="aperture_results.csv")
    ap.add_argument("--plot", help="CSV dei risultati da riassumere")
    ap.add_argument("--png", default="")
    a = ap.parse_args()
    if a.one:
        res = analyse_one(a.one)
        df = pd.DataFrame([res])
        if os.path.exists(a.out):   # unione delle colonne: righe con colonne diverse restano leggibili
            df = pd.concat([pd.read_csv(a.out), df], ignore_index=True)
        df.to_csv(a.out, index=False)
        print(", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in res.items()))
    if a.plot:
        plot(a.plot, a.png)


if __name__ == "__main__":
    main()
