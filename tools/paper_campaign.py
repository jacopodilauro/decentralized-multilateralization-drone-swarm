#!/usr/bin/env python3
"""Campagna di simulazioni per il paper (PAPER_NOTES 4.33): tre esperimenti confrontabili tra loro.

  interni  attaccanti INTERNI: k droni dello sciame dichiarano un GPS falso (15 m, rampa di 10 s), N = 5..50 droni
           (5 e 6 mostrano il limite N >= k + 5: il filtro aggiorna un drone solo con almeno 4 misure, PAPER_NOTES 4.33),
           k = 1, 2, 3 e il 25%, 33%, 45% di N; attaccanti "coordinati" (tutti spostati verso nord) oppure
           "indipendenti" (ognuno in una direzione diversa)
  esterno  spoofer GNSS ESTERNO a singola antenna (a terra, 2 m), distanza R = 25..1000 m, N = 5..50 droni;
           sensori di bordo di livello B (buoni) e, per N = 8 e 20, di livello D (scarsi)
  misto    prima k = 1 o 2 attaccanti interni (t = 70 s), poi lo spoofer esterno (t = 140 s), N = 8 e 20, R = 50, 200 m

Tutto il resto e' IDENTICO in ogni run (confrontabilita'):
  - geometria `--formation=shell`: drone 0 fermo al centro, gli altri su un guscio sferico con 12 m dal vicino
    piu' prossimo qualunque sia N, rotazione rigida a 1 m/s all'equatore, centro a 50 m di quota;
  - radio UWB con perdite (`--radio=uwb`), errore UWB realistico (`--uwbError=real`), ricevitore GNSS realistico;
  - durata 240 s; stessi seed per tutte le configurazioni.

Uso (dalla cartella del repository):
  SIM=/percorso/eseguibile python3 tools/paper_campaign.py --profilo completo --jobs 8
  python3 tools/paper_report.py --out runs/paper
Profili: prova (6 run, verifica), rapido (N <= 20, seed 1-5, ~1 h con 8 core), completo (N fino a 50, seed 1-10;
per N = 50 seed 1-5 e k ridotti; ~5 h con 8 core). Le run piu' economiche vengono eseguite per prime; la campagna e'
riprendibile: rilanciando lo stesso comando si fanno solo le run mancanti (anche dopo Ctrl-C).
"""
import argparse
import contextlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

LEVELS = {"A": (0.0, 0.0, 0.0), "B": (0.1, 0.01, 1.0), "C": (0.2, 0.02, 2.0), "D": (0.3, 0.03, 3.0)}
COMMON = ["--netanim=0", "--verbose=1", "--setnGuests=0", "--formation=shell", "--spacing=12", "--shellSpeed=1",
          "--radio=uwb", "--uwbError=real", "--gnss=receiver", "--setSimTime=240", "--anchorVel=gps"]
# --anchorVel=gps (PAPER_NOTES 4.41): con la velocita' dell'EKF (storico) il filtro diverge con 30-50 droni
T_ATTACK, T_MIX_IN, T_MIX_OUT = 100.0, 70.0, 140.0
BIG = ("tdma_security_log.csv", "gnss_log.csv", "pps_log.csv", "ground_truth.csv", "uwb_log.csv")


def insider_ids(n, k):
    """k attaccanti distribuiti uniformemente sul guscio (ID 1..N-1; il drone 0 al centro resta onesto)."""
    return sorted({1 + (j * (n - 1)) // k for j in range(k)})


def k_values(n):
    ks = {1, 2, 3} | {max(1, round(f * n)) for f in (0.25, 1.0 / 3.0, 0.45)}
    return sorted(k for k in ks if k <= n - 2)


def configs(profile, exps):
    if profile == "prova":
        sizes, seeds = [5, 8], [1]
    elif profile == "rapido":
        sizes, seeds = [5, 6, 8, 12, 20], [1, 2, 3, 4, 5]
    else:
        sizes, seeds = [5, 6, 8, 12, 20, 30, 50], list(range(1, 11))
    out = []
    for n in sizes:
        sd = seeds[:5] if (profile == "completo" and n >= 50) else seeds
        if "interni" in exps:
            ks = k_values(n)
            if profile == "completo" and n >= 50:
                ks = [k for k in ks if k in (1, 3) or k >= round(n / 3.0)]
            for k in ks:
                for mode in ("coordinati", "indipendenti"):
                    if profile == "prova" and not ((n == 5 and mode == "indipendenti") or (n == 8 and k == 1 and mode == "coordinati")):
                        continue
                    for s in sd:
                        out.append(dict(exp="interni", N=n, k=k, mode=mode, R=None, level="B", seed=s))
        if "esterno" in exps:
            for R in (25, 50, 100, 200, 500, 1000):
                for level in ("B", "D"):
                    if level == "D" and n not in (8, 20):
                        continue
                    if profile == "prova" and not ((n == 5 and R == 50 and level == "B") or (n == 8 and R == 200 and level == "B")):
                        continue
                    for s in sd:
                        out.append(dict(exp="esterno", N=n, k=0, mode=None, R=R, level=level, seed=s))
        if "misto" in exps and n in (8, 20):
            for k in (1, 2):
                for R in (50, 200):
                    if profile == "prova" and not (n == 8 and k == 1 and R == 50):
                        continue
                    for s in sd:
                        out.append(dict(exp="misto", N=n, k=k, mode="indipendenti", R=R, level="B", seed=s))
    return out


def label(c):
    if c["exp"] == "interni":
        return f"N{c['N']}_k{c['k']}_{c['mode']}"
    if c["exp"] == "esterno":
        return f"N{c['N']}_R{c['R']}_{c['level']}"
    return f"N{c['N']}_k{c['k']}_R{c['R']}"


def cost_s(c):
    """Stima del tempo di una run [s] su un PC desktop (20 droni ~80 s); cresce circa come N^2,6."""
    return 0.0332 * c["N"] ** 2.6 + 8.0 + (0.4 * c["N"] if c["exp"] != "interni" else 0.0)


def sim_args(c):
    b, sc, h = LEVELS[c["level"]]
    a = COMMON + [f"--runId={c['seed']}", f"--setnDrones={c['N']}",
                  f"--imuVelBias={b}", f"--imuScale={sc}", f"--imuHeadingDeg={h}"]
    ids = ",".join(str(i) for i in insider_ids(c["N"], c["k"])) if c["k"] > 0 else ""
    if c["exp"] == "interni":
        a += [f"--targetsId={ids}", f"--attackTime={T_ATTACK}", f"--insiderSpread={int(c['mode'] == 'indipendenti')}"]
    elif c["exp"] == "esterno":
        a += ["--spoof=1", f"--spoofTime={T_ATTACK}", f"--spoofDist={c['R']}"]
    else:
        a += ["--spoof=1", "--mixed=1", f"--spoofTime={T_MIX_OUT}", f"--spoofDist={c['R']}",
              f"--targetsId={ids}", f"--attackTime={T_MIX_IN}", "--insiderSpread=1"]
    return a


def parse_stdout(path):
    r = {}
    txt = open(path, errors="replace").read()
    for key, pat in (("raggio_guscio_m", r"raggio ([\d.]+) m"), ("frame_ms", r"frame TDMA ([\d.]+) ms"),
                     ("pacchetto_max_byte", r"dimensione massima (\d+) byte"),
                     ("uwb_ricezione_pct", r"Radio UWB: pacchetti ricevuti ([\d.]+)%")):
        m = re.search(pat, txt)
        r[key] = float(m.group(1)) if m else np.nan
    r["droni_consenso_cattura"] = len(set(re.findall(r"drone (\d+): CONSENSO: sciame catturato", txt)))
    r["espulsioni"] = len(re.findall(r"Espulsione forzata", txt))
    return r


def insider_summary(m, ids):
    """Aggregati sugli attaccanti interni da metrics.compute (stesse definizioni dei risultati precedenti)."""
    det, lat, cov = 0, [], []
    for t in ids:
        x = m.get("rilevamento", {}).get(str(t))
        if not x or not x.get("osservatori_che_lo_vedono"):
            continue
        f = x["osservatori_che_rilevano"] / x["osservatori_che_lo_vedono"]
        cov.append(f)
        if f >= 0.5:
            det += 1
            if x.get("latenza_mediana_s") is not None:
                lat.append(x["latenza_mediana_s"])
    return {"attaccanti_rilevati_pct": 100.0 * det / len(ids) if ids else np.nan,
            "copertura_osservatori_pct": 100.0 * float(np.mean(cov)) if cov else np.nan,
            "latenza_rilevamento_s": float(np.median(lat)) if lat else np.nan}


def scalar_metrics(m, prefix=""):
    out = {}
    for k, v in m.items():
        if k in ("rilevamento", "fa_pre_per_mittente"):
            continue
        if isinstance(v, dict):
            out.update(scalar_metrics(v, prefix + k + "."))
        elif isinstance(v, (int, float)) and v is not None and not isinstance(v, bool):
            out[prefix + k] = float(v)
    return out


# Analisi in parallelo limitate da un semaforo: ogni analisi carica in memoria il log della run (milioni di righe con
# 30-50 droni); con molte analisi insieme la RAM si esaurisce e Linux uccide un processo (tutta la coda fallisce).
_ANALISI = None


def _init_worker(sem):
    global _ANALISI
    _ANALISI = sem


def _analyse(c, d, rc, row):
    import metrics          # noqa: E402  (import nel processo figlio)
    import aperture_sweep   # noqa: E402
    try:
        if rc != 0:
            raise RuntimeError(f"il simulatore e' uscito con codice {rc}")
        row.update(parse_stdout(os.path.join(d, "stdout.txt")))
        tdma = os.path.join(d, "tdma_security_log.csv")
        ids = insider_ids(c["N"], c["k"]) if c["k"] > 0 else []
        t_in = T_ATTACK if c["exp"] == "interni" else T_MIX_IN
        if ids:
            m = metrics.compute(tdma, t_in, ids, c["N"])
            row.update(insider_summary(m, ids))
            row.update({"int." + k: v for k, v in scalar_metrics(m).items()})
        if c["exp"] != "interni":
            t_out = T_ATTACK if c["exp"] == "esterno" else T_MIX_OUT
            mc = metrics.compute(tdma, t_out, [], c["N"])
            row.update({"est." + k: v for k, v in scalar_metrics(mc).items()})
            a = aperture_sweep.analyse_one(d)
            row.update({"loc." + k: v for k, v in a.items() if k != "run"})
        row["ok"] = 1
    except Exception as e:  # la run resta registrata, con l'errore
        row["ok"] = 0
        row["errore"] = str(e)[:300]


def run_one(c, sim, out):
    d = os.path.join(out, c["exp"], label(c), f"seed{c['seed']}")
    rowf = os.path.join(d, "row.json")
    if os.path.exists(rowf):
        return json.load(open(rowf))
    if os.path.isdir(d):
        shutil.rmtree(d)
    os.makedirs(d)
    t0 = time.time()
    with open(os.path.join(d, "stdout.txt"), "w") as fh:
        rc = subprocess.call([sim] + sim_args(c), cwd=d, stdout=fh, stderr=subprocess.STDOUT)
    row = dict(c)
    row["insider_ids"] = " ".join(str(i) for i in insider_ids(c["N"], c["k"])) if c["k"] > 0 else ""
    row["tempo_run_s"] = round(time.time() - t0, 1)
    row["rc"] = rc
    with (_ANALISI if _ANALISI is not None else contextlib.nullcontext()):
        _analyse(c, d, rc, row)
    for f in BIG:
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)
    row = {k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in row.items()}
    json.dump(row, open(rowf, "w"), default=lambda o: o.item() if hasattr(o, "item") else str(o))
    return row


def write_tables(out, exps):
    import pandas as pd
    for e in exps:
        rows = []
        base = os.path.join(out, e)
        if not os.path.isdir(base):
            continue
        for root, _, files in os.walk(base):
            if "row.json" in files:
                rows.append(json.load(open(os.path.join(root, "row.json"))))
        if rows:
            pd.DataFrame(rows).sort_values(["N", "k", "R", "seed"], na_position="first").to_csv(
                os.path.join(base, "risultati.csv"), index=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exp", default="tutti", choices=["tutti", "interni", "esterno", "misto"])
    ap.add_argument("--profilo", default="completo", choices=["prova", "rapido", "completo"])
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--out", default="runs/paper")
    ap.add_argument("--analisi", type=int, default=1,
                    help="analisi dei log in parallelo (ognuna carica il log intero: con 30-50 droni tenere 1)")
    ap.add_argument("--stima", action="store_true", help="stampa solo il numero di run e il tempo stimato")
    a = ap.parse_args()
    exps = ["interni", "esterno", "misto"] if a.exp == "tutti" else [a.exp]
    sim = os.environ.get("SIM")
    cs = sorted(configs(a.profilo, exps), key=cost_s)
    done = [c for c in cs if os.path.exists(os.path.join(a.out, c["exp"], label(c), f"seed{c['seed']}", "row.json"))]
    todo = [c for c in cs if c not in done]
    est = sum(cost_s(c) for c in todo) / max(1, a.jobs) / 3600.0
    print(f">>> profilo {a.profilo}, esperimenti {', '.join(exps)}: {len(cs)} run ({len(done)} gia' fatte), "
          f"stima ~{est:.1f} h con {a.jobs} processi")
    for e in exps:
        print(f"    {e}: {sum(c['exp'] == e for c in cs)} run")
    if a.stima:
        return
    if not sim or not os.path.exists(sim):
        sys.exit("imposta SIM=/percorso/eseguibile del simulatore")
    sim = os.path.abspath(sim)
    # Una cartella = una sola versione delle opzioni comuni: riprendere con opzioni diverse mescolerebbe run non
    # confrontabili (le run gia' fatte vengono saltate guardando solo row.json)
    cf = os.path.join(a.out, "opzioni_comuni.json")
    if os.path.exists(cf) and json.load(open(cf)) != COMMON:
        sys.exit(f"{a.out} contiene run con opzioni comuni diverse ({cf}): usa un'altra --out")
    if not os.path.exists(cf) and done:
        sys.exit(f"{a.out} contiene run senza {os.path.basename(cf)} (campagna precedente): usa un'altra --out")
    os.makedirs(a.out, exist_ok=True)
    json.dump(COMMON, open(cf, "w"))
    t0, n_ok, n_err, rotto = time.time(), 0, 0, False
    # Ogni processo di lavoro fa UNA run e poi viene sostituito (max_tasks_per_child=1, richiede "spawn"): un processo
    # Python che ha analizzato un log di milioni di righe non restituisce tutta la memoria al sistema, e con i processi
    # riusati la RAM occupata cresceva a ogni run fino a far uccidere un processo (con 30-50 droni). I risultati non cambiano.
    ctx = mp.get_context("spawn")
    sem = ctx.Semaphore(max(1, a.analisi))
    kw = dict(max_workers=a.jobs, mp_context=ctx, initializer=_init_worker, initargs=(sem,))
    try:
        pool = ProcessPoolExecutor(max_tasks_per_child=1, **kw)
    except TypeError:   # Python < 3.11: nessun riciclo dei processi
        pool = ProcessPoolExecutor(**kw)
    with pool as ex:
        fut = {ex.submit(run_one, c, sim, a.out): c for c in todo}
        for i, f in enumerate(as_completed(fut), 1):
            c = fut[f]
            try:
                r = f.result()
                ok = r.get("ok", 0) == 1
            except Exception as e:
                ok, r = False, {"errore": str(e)}
                rotto = rotto or "terminated abruptly" in str(e)
            n_ok += ok
            n_err += (not ok)
            el = (time.time() - t0) / 60.0
            print(f"    [{i}/{len(todo)}, {el:.1f} min] {c['exp']} {label(c)} seed {c['seed']}: "
                  f"{'ok' if ok else 'ERRORE ' + str(r.get('errore', ''))[:120]}", flush=True)
            if i % 10 == 0 or i == len(todo):
                write_tables(a.out, exps)
    write_tables(a.out, exps)
    print(f">>> fatto: {n_ok} run riuscite, {n_err} con errore; tabelle in {a.out}/<esperimento>/risultati.csv")
    if rotto:
        print(">>> ATTENZIONE: un processo e' stato ucciso dal sistema (quasi sempre per memoria esaurita) e le run in "
              "coda sono fallite senza partire. Controlla con: journalctl -k | grep -i 'killed process'. "
              "Chiudi le simulazioni rimaste attive: pkill -f " + os.path.basename(sim) + ". "
              "Poi rilancia lo stesso comando (anche con meno --jobs): le run gia' fatte restano.")


if __name__ == "__main__":
    main()
