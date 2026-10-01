#!/usr/bin/env python3
"""
Confronta due file di metriche (singolo run o aggregati) e dice cosa e' cambiato.

Uso:
    python3 tools/compare.py runs/prima/summary.json runs/dopo/summary.json

Esce con codice 0 se tutto e' identico, 1 se qualcosa e' cambiato.
Con --tol si accettano differenze relative piccole (es. --tol 0.01 = 1%).
"""
import argparse
import json
import math
import sys


def load(path):
    m = json.load(open(path))
    flat = {}

    def walk(d, prefix=""):
        for k, v in d.items():
            key = f"{prefix}{k}"
            if isinstance(v, dict):
                # file aggregato: usa la media
                if "media" in v and "std" in v:
                    flat[key] = (v["media"], v["std"])
                else:
                    walk(v, key + ".")
            elif isinstance(v, (int, float)) and v is not None:
                flat[key] = (float(v), None)
    walk(m)
    return flat


def fmt(x):
    if x is None:
        return "-"
    if abs(x) >= 1000:
        return f"{x:.0f}"
    return f"{x:.4g}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prima")
    ap.add_argument("dopo")
    ap.add_argument("--tol", type=float, default=0.0,
                    help="tolleranza relativa (0 = devono essere identici)")
    a = ap.parse_args()

    A, B = load(a.prima), load(a.dopo)
    keys = sorted(set(A) | set(B))
    changed = 0
    w = max(len(k) for k in keys)
    print(f"{'metrica'.ljust(w)}  {'prima':>12}  {'dopo':>12}  esito")
    print("-" * (w + 40))
    for k in keys:
        va = A.get(k, (None, None))[0]
        vb = B.get(k, (None, None))[0]
        if va is None or vb is None:
            status = "NUOVA" if va is None else "RIMOSSA"
            changed += 1
        else:
            den = max(abs(va), abs(vb), 1e-12)
            rel = abs(va - vb) / den
            if (math.isnan(va) and math.isnan(vb)) or rel <= a.tol:
                status = "="
            else:
                status = f"CAMBIATA ({(vb - va):+.4g})"
                changed += 1
        print(f"{k.ljust(w)}  {fmt(va):>12}  {fmt(vb):>12}  {status}")
    print("-" * (w + 40))
    print("Nessuna differenza." if changed == 0 else f"{changed} metriche diverse.")
    return 0 if changed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
