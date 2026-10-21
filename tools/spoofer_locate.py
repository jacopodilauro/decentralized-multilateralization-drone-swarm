#!/usr/bin/env python3
"""Localizzazione dello spoofer con la geometria STIMATA dallo sciame (Fase 9, primo passo, analisi dei log).

Uso: python3 tools/spoofer_locate.py runs/<etichetta>/seed1 [--observer 1] [--png figura.png] [--legacy]

Dal 9.1, se la run ha pps_log.csv: differenze di PPS misurate davvero (PPS con quantizzazione/rumore del
ricevitore, marcato con l'orologio UWB, riportato nell'orologio dell'osservatore con la relazione tra orologi
stimata dai pacchetti); TDoA con una costante per secondo. Colonne: geometria e sync ideali | geometria stimata +
sync ideale | geometria stimata + sync REALE (nessuna ipotesi ideale). Con --legacy l'analisi del 9.0/8.2c:

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


def solve_tdoa_epochs(P, d, ep):
    """TDoA con una costante incognita per epoca (secondo): d_k = |p_k - s| - |p_ref(ep) - s| + cost(ep).
    Sottraendo la media di ogni epoca la costante sparisce. Gauss-Newton da 12 direzioni a 300 m poco sotto la quota
    dello sciame (partenze storiche) e da 6 direzioni a 40 m, poco sotto e 45 gradi sotto: con lo spoofer vicino e a
    terra, partendo solo da 300 m la ricerca scivolava verso soluzioni lontanissime (PAPER_NOTES 4.30).
    Vince la soluzione con il residuo minore."""
    c0 = P.mean(axis=0)
    _, inv = np.unique(ep, return_inverse=True)
    cnt = np.bincount(inv)
    dm = d - (np.bincount(inv, d) / cnt)[inv]
    best, best_res = None, np.inf
    starts = [(300.0, -0.1, a) for a in range(12)]                                     # partenze storiche (prime)
    starts += [(40.0, kz, 2 * a) for kz in (-0.1, -1.0) for a in range(6)]               # vicine, 6 direzioni
    for r0, kz, a in starts:
        s = c0 + r0 * np.array([np.cos(a * np.pi / 6), np.sin(a * np.pi / 6), kz])
        for _ in range(40):
            v = s - P
            rho = np.linalg.norm(v, axis=1)
            u = v / rho[:, None]
            res = dm - (rho - (np.bincount(inv, rho) / cnt)[inv])
            H = u - np.stack([np.bincount(inv, u[:, i]) / cnt for i in range(3)], axis=1)[inv]
            dx = np.linalg.lstsq(H, res, rcond=None)[0]
            s = s + dx
            if np.linalg.norm(s - c0) > 20000:
                s = c0 + 20000 * (s - c0) / np.linalg.norm(s - c0)
            if np.linalg.norm(dx) < 1e-3:
                break
        rho = np.linalg.norm(s - P, axis=1)
        rr = np.sum((dm - (rho - (np.bincount(inv, rho) / cnt)[inv])) ** 2)
        if rr < best_res:
            best, best_res = s.copy(), rr
    return best


def pps_mode(a, s_true, d):
    """9.1: differenze di PPS misurate nell'orologio UWB dell'osservatore (pps_log.csv): sincronizzazione reale."""
    pl = pd.read_csv(os.path.join(a.rundir, "pps_log.csv"))
    pl = pl[pl.observer == a.observer].sort_values("time")
    o = d[(d.observer_id == a.observer) & (d.capture_consensus == 1)].sort_values("time")
    rows = []
    for j, pj in pl.groupby("sender"):
        oj = o[o.sender_id == j][["time", "est_x", "est_y", "est_z", "true_x", "true_y", "true_z"]]
        if oj.empty:
            continue
        pj = pj.assign(tq=pj.sec.astype(float))   # posizione all'istante del PPS (secondo intero)
        m = pd.merge_asof(pj.sort_values("tq"), oj.rename(columns={"time": "tq"}), on="tq",
                          direction="nearest", tolerance=0.2)
        rows.append(m.dropna(subset=["est_x"]))
    m = pd.concat(rows).sort_values("sec")
    m = m[m.groupby("sec").sender.transform("size") >= 4]
    E = m[["est_x", "est_y", "est_z"]].values
    T = m[["true_x", "true_y", "true_z"]].values
    ep = m.sec.values
    sync = np.std(m.d_meas_m - m.d_ideal_m)
    print(f"Spoofer vero {np.round(s_true, 1)}; osservatore {a.observer}; {m.sender.nunique()} droni, {len(m)} differenze "
          f"di PPS; errore di sincronizzazione (misura - ideale) std {sync:.2f} m")
    print("Nessuna ipotesi ideale nella colonna di destra (geometria stimata, PPS e orologi UWB reali).\n")
    print(f"{'dopo':>6s} | {'geometria e sync ideali':^27s} | {'stimata + sync ideale':^27s} | "
          f"{'stimata + sync REALE':^27s}")
    t0 = ep.min()
    out = []
    for T_end in np.arange(5, ep.max() - t0 + 1e-6, 5.0):
        k = ep <= t0 + T_end
        if np.unique(ep[k]).size < 3:
            continue
        c = T[k & (ep > t0 + T_end - 5)].mean(axis=0)
        res = [T_end]
        for P_, dd in ((T, m.d_ideal_m.values), (E, m.d_ideal_m.values), (E, m.d_meas_m.values)):
            res += list(errors(solve_tdoa_epochs(P_[k], dd[k], ep[k]), c, s_true))
        out.append(res)
        print(f"{T_end:5.0f}s | {res[1]:+7.2f}° {res[2]:+6.2f}° {res[3]:+8.0f}m | {res[4]:+7.2f}° {res[5]:+6.2f}° "
              f"{res[6]:+8.0f}m | {res[7]:+7.2f}° {res[8]:+6.2f}° {res[9]:+8.0f}m")
    out = np.array(out)
    late = out[:, 0] >= 60
    for name, k in (("geometria e sync ideali", 1), ("stimata + sync ideale", 4), ("stimata + sync REALE", 7)):
        print(f"  da 60 s in poi, {name:24s}: |azimut| mediano {np.median(np.abs(out[late, k])):5.2f}°, "
              f"|elevazione| {np.median(np.abs(out[late, k + 1])):5.2f}°, |distanza| mediana "
              f"{np.median(np.abs(out[late, k + 2])):6.0f} m")
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for k, lab, st in ((1, "geometria e sincronizzazione ideali", "k--"),
                           (4, "geometria stimata dallo sciame, sincronizzazione ideale", "C0-"),
                           (7, "geometria stimata + sincronizzazione reale (PPS + orologi UWB)", "C3-")):
            ax[0].plot(out[:, 0], np.abs(out[:, k]), st, label=lab)
            ax[1].plot(out[:, 0], np.abs(out[:, k + 2]), st, label=lab)
        ax[0].set_ylabel("errore di azimut [gradi]"); ax[1].set_ylabel("errore di distanza [m]")
        ax[0].set_yscale("log"); ax[1].set_yscale("log")
        for x_ in ax:
            x_.set_xlabel("secondi dall'inizio della localizzazione"); x_.grid(alpha=.3)
        ax[0].legend(fontsize=7)
        fig.suptitle(f"Localizzazione dello spoofer dai PPS dello sciame catturato (osservatore {a.observer}, "
                     f"jitter PPS reale: errore di sincronizzazione {sync:.1f} m)", fontsize=10)
        fig.tight_layout(); fig.savefig(a.png, dpi=150)
        print(f"\nFigura: {a.png}")


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
    ap.add_argument("--legacy", action="store_true", help="analisi del 9.0/8.2c (PPS ideali dal gnss_log)")
    a = ap.parse_args()

    s_true, t0 = antenna_from_stdout(os.path.join(a.rundir, "stdout.txt"))
    g = pd.read_csv(os.path.join(a.rundir, "gnss_log.csv"))
    d = pd.read_csv(os.path.join(a.rundir, "tdma_security_log.csv"),
                    usecols=["time", "observer_id", "sender_id", "est_x", "est_y", "est_z",
                             "true_x", "true_y", "true_z", "capture_consensus"])
    if os.path.exists(os.path.join(a.rundir, "pps_log.csv")) and not a.legacy:
        return pps_mode(a, s_true, d)
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
    print("Ipotesi ideali: sincronizzazione (PPS dal gnss_log); solo la colonna '8.2c ideale' e' allineata alla verita'.\n")
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
