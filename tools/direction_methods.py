#!/usr/bin/env python3
"""Direzione dello spoofer: confronto di tre metodi sulle stesse misure (tutto reale: geometria stimata dallo
sciame, PPS e orologi UWB reali).

Uso: python3 tools/direction_methods.py runs/<etichetta>/seed<N> [--out risultati.csv]

  cumulativo   (metodo attuale) minimi quadrati su posizione dello spoofer con tutte le differenze di PPS
               accumulate dal consenso alla fine (posizioni stimate in istanti diversi, riferimenti ruotati in
               modo diverso);
  per secondo  a ogni secondo un'onda piana: d_j - media = -(p_j - media) . g, g = direzione verso lo spoofer
               (la curvatura e la distanza non servono), nel riferimento di quel secondo; si scartano i secondi
               con |g| lontano da 1 (controllo di qualita' che NON usa la verita': per un'onda piana |g| = 1);
               direzione finale = mediana delle direzioni per secondo;
  fusione      ogni drone annuncia la propria direzione; lo sciame prende la mediana sui droni base (per
               ciascuno dei due metodi sopra).
La verita' (antenna dello spoofer, posizioni vere) serve solo a calcolare l'errore finale.
"""
import argparse, os, sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spoofer_view import antenna_from_stdout  # noqa: E402
from spoofer_locate import solve_tdoa_epochs  # noqa: E402
from aperture_sweep import observer_table  # noqa: E402

N_BASE = 8


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def circ_median(az):
    """Mediana di angoli [gradi]: srotolati attorno alla media circolare."""
    az = np.asarray(az)
    ref = np.degrees(np.arctan2(np.mean(np.sin(np.radians(az))), np.mean(np.cos(np.radians(az)))))
    return wrap(ref + np.median(wrap(az - ref)))


def per_second_azimuth(E, d, ep, gmin=0.7, gmax=1.3):
    az = []
    for N in np.unique(ep):
        k = ep == N
        if k.sum() < 5:
            continue
        X = E[k] - E[k].mean(axis=0)
        y = d[k] - d[k].mean()
        g = np.linalg.lstsq(-X, y, rcond=None)[0]
        n = np.linalg.norm(g)
        if gmin <= n <= gmax:
            az.append(np.degrees(np.arctan2(g[1], g[0])))
    return (circ_median(az), len(az)) if az else (np.nan, 0)


def analyse(rundir):
    s, _ = antenna_from_stdout(os.path.join(rundir, "stdout.txt"))
    pl = pd.read_csv(os.path.join(rundir, "pps_log.csv"))
    d = pd.read_csv(os.path.join(rundir, "tdma_security_log.csv"),
                    usecols=["time", "observer_id", "sender_id", "est_x", "est_y", "est_z",
                             "true_x", "true_y", "true_z", "capture_consensus"])
    rows = []
    for o in range(N_BASE):
        m = observer_table(pl, d, o)
        if m is None or m.sec.nunique() < 10:
            continue
        E = m[["est_x", "est_y", "est_z"]].values
        T = m[["true_x", "true_y", "true_z"]].values
        dm = m.d_meas_m.values
        ep = m.sec.values
        c = T.mean(axis=0)                                   # centro vero dello sciame durante la cattura
        az_true = np.degrees(np.arctan2(s[1] - c[1], s[0] - c[0]))
        x = solve_tdoa_epochs(E, dm, ep)
        az_cum = np.degrees(np.arctan2(x[1] - c[1], x[0] - c[0]))
        az_sec, n_ok = per_second_azimuth(E, dm, ep)
        rows.append({"oss": o, "az_vero": az_true, "az_cumulativo": az_cum, "az_per_secondo": az_sec,
                     "secondi_usati": n_ok, "secondi_totali": int(np.unique(ep).size)})
    r = pd.DataFrame(rows)
    r["err_cumulativo"] = wrap(r.az_cumulativo - r.az_vero)
    r["err_per_secondo"] = wrap(r.az_per_secondo - r.az_vero)
    az_true = circ_median(r.az_vero)
    fused = {"fusione_cumulativo": wrap(circ_median(r.az_cumulativo) - az_true),
             "fusione_per_secondo": wrap(circ_median(r.az_per_secondo.dropna()) - az_true)}
    return r, fused


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rundir")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    r, fused = analyse(a.rundir)
    print(r[["oss", "err_cumulativo", "err_per_secondo", "secondi_usati", "secondi_totali"]].round(2).to_string(index=False))
    print(f"  |errore| mediano sui droni: cumulativo {np.median(abs(r.err_cumulativo)):.2f} gradi, per secondo "
          f"{np.nanmedian(abs(r.err_per_secondo)):.2f} gradi")
    print(f"  fusione (mediana sui droni): cumulativo {fused['fusione_cumulativo']:+.2f} gradi, per secondo "
          f"{fused['fusione_per_secondo']:+.2f} gradi")
    if a.out:
        row = {"run": a.rundir,
               "cum_mediana": float(np.median(abs(r.err_cumulativo))), "cum_max": float(abs(r.err_cumulativo).max()),
               "sec_mediana": float(np.nanmedian(abs(r.err_per_secondo))), "sec_max": float(np.nanmax(abs(r.err_per_secondo))),
               "fus_cum": float(fused["fusione_cumulativo"]), "fus_sec": float(fused["fusione_per_secondo"])}
        df = pd.DataFrame([row])
        if os.path.exists(a.out):
            df = pd.concat([pd.read_csv(a.out), df], ignore_index=True)
        df.to_csv(a.out, index=False)


if __name__ == "__main__":
    main()
