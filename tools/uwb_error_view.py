#!/usr/bin/env python3
"""Errore delle distanze DS-TWR contro la verita' (Fase 5), dal log `uwb_log.csv` (opzione --uwbLog=1).

Uso:
  python3 tools/uwb_error_view.py runs/<etichetta>/seed1 [--png figura.png]

Stampa: frazione NLOS vera, matrice di confusione del rivelatore NLOS (sulle distanze DS-TWR), media e deviazione
standard dell'errore per stato vero e dichiarato, deviazione standard dell'errore LOS per fasce di distanza.

Nota: la distanza vera e' quella al messaggio centrale ricostruita con le velocita' vere (errore ~2 cm per gli ospiti
in curva); lo stato vero e' NLOS se almeno una delle tre ricezioni lo era (per la terza si usa l'ultimo stato del
collegamento inverso).
"""
import argparse
import csv
import math
import os
import sys


def stats(v):
    n = len(v)
    if n == 0:
        return 0, float("nan"), float("nan")
    m = sum(v) / n
    s = math.sqrt(sum((x - m) ** 2 for x in v) / n) if n > 1 else 0.0
    return n, m, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="cartella della simulazione (con uwb_log.csv)")
    ap.add_argument("--bin", type=float, default=10.0, help="larghezza delle fasce di distanza [m]")
    ap.add_argument("--png", default="", help="salva la figura errore LOS contro distanza")
    a = ap.parse_args()
    path = os.path.join(a.run, "uwb_log.csv")
    if not os.path.exists(path):
        sys.exit(f"{path} non trovato: esegui la simulazione con --uwbLog=1")

    groups = {(t, f): [] for t in (0, 1) for f in (0, 1)}   # (nlos vero, dichiarato NLOS) -> errori
    bins = {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            e = float(r["d_meas_m"]) - float(r["d_true_m"])
            nl = int(r["nlos_true"])
            flagN = 1 - int(r["los_flag"])
            groups[(nl, flagN)].append(e)
            if nl == 0:
                b = int(float(r["d_true_m"]) // a.bin)
                bins.setdefault(b, []).append(e)

    tot = sum(len(v) for v in groups.values())
    nN = len(groups[(1, 0)]) + len(groups[(1, 1)])
    nL = len(groups[(0, 0)]) + len(groups[(0, 1)])
    print(f"distanze DS-TWR: {tot}, NLOS vere {100.0 * nN / max(tot, 1):.2f}%")
    print(f"rivelatore: LOS dichiarate NLOS {100.0 * len(groups[(0, 1)]) / max(nL, 1):.2f}%, "
          f"NLOS dichiarate LOS (non rilevate) {100.0 * len(groups[(1, 0)]) / max(nN, 1):.2f}%")
    names = {(0, 0): "LOS vera, dichiarata LOS", (0, 1): "LOS vera, dichiarata NLOS",
             (1, 0): "NLOS vera, dichiarata LOS", (1, 1): "NLOS vera, dichiarata NLOS"}
    for k in [(0, 0), (0, 1), (1, 0), (1, 1)]:
        n, m, s = stats(groups[k])
        rms = math.sqrt(sum(x * x for x in groups[k]) / n) if n else float("nan")
        print(f"  {names[k]:28s} n={n:8d}  media {100 * m:7.2f} cm  dev.std {100 * s:6.2f} cm  rms {100 * rms:6.2f} cm")
    print(f"errore LOS per fascia di distanza ({a.bin:.0f} m):")
    xs, ys = [], []
    for b in sorted(bins):
        n, m, s = stats(bins[b])
        if n < 200:
            continue
        print(f"  {b * a.bin:5.0f}-{(b + 1) * a.bin:5.0f} m: n={n:8d}  media {100 * m:6.2f} cm  dev.std {100 * s:6.2f} cm")
        xs.append((b + 0.5) * a.bin); ys.append(100 * s)
    if a.png and xs:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(5, 3.2))
        plt.plot(xs, ys, "o-")
        plt.xlabel("distanza vera [m]")
        plt.ylabel("dev. std errore DS-TWR LOS [cm]")
        plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(a.png, dpi=150)
        print(f"figura: {a.png}")


if __name__ == "__main__":
    main()
