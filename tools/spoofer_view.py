#!/usr/bin/env python3
"""Dati dello spoofer esterno da una run con --spoof=1 (passo 7.2).

Uso: python3 tools/spoofer_view.py runs/<etichetta>/seed1 [--jitter-ns 5] [--pps-hz 1]

Legge gnss_log.csv (un fix GPS per riga), stdout.txt (posizione dell'antenna) e, se c'e',
tdma_security_log.csv. Stampa:
  1) cattura: quando i droni vengono catturati e se vedono tutti la stessa posizione falsa;
  2) reazione del sistema attuale (allarmi collettivi, chi-quadro) dopo la cattura;
  3) localizzazione dello spoofer dai PPS: ogni drone catturato da' r_i = |p_i - s| + c*beta (+ rumore);
     minimi quadrati su (s, c*beta) con i PPS accumulati da quando inizia lo spoofing.
     ATTENZIONE: qui posizioni dei droni e sincronizzazione sono IDEALI (verita'); nella Fase 9 verranno
     dalla geometria UWB dello sciame e dagli offset tra orologi UWB.
"""
import argparse, os, re, sys
import numpy as np
import pandas as pd


def antenna_from_stdout(path):
    with open(path) as f:
        for line in f:
            m = re.search(r"antenna \(\s*(\S+)\s+(\S+)\s+(\S+)\s*\) attivo da t=(\S+) s", line)
            if m:
                return np.array([float(m.group(i)) for i in (1, 2, 3)]), float(m.group(4))
    sys.exit("stdout.txt non contiene la riga dello spoofer: la run e' stata fatta con --spoof=1?")


def solve_toa(P, r):
    """Minimi quadrati su (s, b) con r_i = |p_i - s| + b. Gauss-Newton da 12 direzioni a 300 m."""
    c0 = P.mean(axis=0)
    best, best_res = None, np.inf
    for a in range(12):
        x = np.r_[c0 + 300 * np.array([np.cos(a * np.pi / 6), np.sin(a * np.pi / 6), -0.1]), 0.0]
        x[3] = np.median(r - np.linalg.norm(P - x[:3], axis=1))
        for _ in range(40):
            d = x[:3] - P
            rho = np.linalg.norm(d, axis=1)
            H = np.c_[d / rho[:, None], np.ones(len(P))]
            res = r - (rho + x[3])
            dx = np.linalg.lstsq(H, res, rcond=None)[0]
            x += dx
            v = x[:3] - c0
            if np.linalg.norm(v) > 20000:
                x[:3] = c0 + 20000 * v / np.linalg.norm(v)
            if np.linalg.norm(dx[:3]) < 1e-3:
                break
        rr = np.sum((r - (np.linalg.norm(P - x[:3], axis=1) + x[3])) ** 2)
        if rr < best_res:
            best, best_res = x.copy(), rr
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rundir")
    ap.add_argument("--jitter-ns", type=float, default=0.0, help="jitter del PPS aggiunto (1 sigma, ns)")
    ap.add_argument("--pps-hz", type=float, default=1.0, help="frequenza dei PPS usati (1 = realistico)")
    a = ap.parse_args()

    s_true, t0 = antenna_from_stdout(os.path.join(a.rundir, "stdout.txt"))
    g = pd.read_csv(os.path.join(a.rundir, "gnss_log.csv"))
    center = g[g.time < t0][["true_x", "true_y", "true_z"]].mean().values
    v = s_true - center
    print(f"Spoofer: antenna {np.round(s_true, 1)}, attivo da t = {t0:.1f} s, "
          f"a {np.linalg.norm(v[:2]):.0f} m in orizzontale dal centro dello sciame")

    # 1) Cattura
    sp = g[g.spoofed == 1]
    first = sp.groupby("node").time.min()
    print("\n1) CATTURA")
    print(f"   droni catturati: {len(first)}; primo fix falso dopo {first.min() - t0:.3f} s, ultimo dopo "
          f"{first.max() - t0:.3f} s (gli ospiti entrano/escono dal geofence)")
    sp = sp.assign(tb=(sp.time * 10).round())
    spread = sp.groupby("tb")[["gps_x", "gps_y", "gps_z"]].std().pow(2).sum(axis=1).pow(.5)
    true_spread = sp.groupby("tb")[["true_x", "true_y", "true_z"]].std().pow(2).sum(axis=1).pow(.5)
    print(f"   dispersione delle posizioni GPS tra droni (mediana): {spread.median():.2f} m "
          f"(posizioni vere: {true_spread.median():.1f} m) -> tutti vedono lo stesso punto falso")

    # 2) Reazione del sistema attuale
    tl = os.path.join(a.rundir, "tdma_security_log.csv")
    if os.path.exists(tl):
        d = pd.read_csv(tl, usecols=["time", "observer_id", "sender_id", "alarm", "gps_chi2", "pos_std"])
        before = d[(d.time > 20) & (d.time < t0)]
        after = d[d.time >= t0 + 5]
        print("\n2) SISTEMA ATTUALE (pensato per pochi droni che mentono)")
        print(f"   allarmi collettivi: prima {100 * before.alarm.mean():.2f}% delle righe, dopo {100 * after.alarm.mean():.1f}%")
        print(f"   chi-quadro mediano: prima {before.gps_chi2.median():.1f}, dopo {after.gps_chi2.median():.0f} "
              f"(soglia 16,27)")
        print(f"   pos_std mediana: prima {before.pos_std.median():.2f} m, dopo {after.pos_std.median():.2f} m")

    # 3) Localizzazione dai PPS
    rng = np.random.default_rng(1)
    sp = sp.assign(sec=np.floor(sp.time * a.pps_hz))
    pps = sp.groupby(["node", "sec"]).first().reset_index()   # un PPS per drone e per periodo
    P = pps[["true_x", "true_y", "true_z"]].values
    r = pps.pps_err_m.values + 0.299792458 * a.jitter_ns * rng.standard_normal(len(pps))
    print(f"\n3) LOCALIZZAZIONE DAI PPS ({a.pps_hz:g} Hz, jitter {a.jitter_ns:g} ns; geometria e sincronizzazione "
          f"IDEALI)")
    print(f"   {'dopo':>6s} {'PPS':>6s} {'droni':>6s} {'apertura':>9s} {'azimut':>9s} {'elevaz.':>9s} "
          f"{'distanza':>10s} {'stima':>24s}")
    for T in (1, 2, 5, 10, 20, 30, 60, 120):
        w = pps[pps.time < t0 + T]
        if w.empty or w.time.max() < t0 + 0.8 * T:
            continue
        idx = w.index.values
        x = solve_toa(P[idx], r[idx])
        e = x[:3] - center
        az = np.degrees(np.arctan2(e[1], e[0]) - np.arctan2(v[1], v[0]))
        az = (az + 180) % 360 - 180
        el = np.degrees(np.arctan2(e[2], np.linalg.norm(e[:2])) - np.arctan2(v[2], np.linalg.norm(v[:2])))
        ap_ = np.ptp(P[idx], axis=0).max()
        print(f"   {T:5d}s {len(idx):6d} {w.node.nunique():6d} {ap_:8.0f}m {az:+8.2f}° {el:+8.2f}° "
              f"{np.linalg.norm(e) - np.linalg.norm(v):+9.0f}m   ({x[0]:.0f}, {x[1]:.0f}, {x[2]:.0f})")


if __name__ == "__main__":
    main()
