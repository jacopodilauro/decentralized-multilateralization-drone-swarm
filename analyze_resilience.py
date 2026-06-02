"""
analyze_resilience.py  –  SwarmRaft Security Experiment Analyzer
=================================================================
Uso:
    python analyze_resilience.py --csv tdma_security_log.csv \
                                  --attack_time 200.0 \
                                  --malicious 0,1 \
                                  --n_drones 8

Output:
    - Tabella riassuntiva su stdout
    - resilience_report.csv  (append, una riga per attaccante per run)
    - resilience_plots.png
"""

import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ─────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--csv",         default="tdma_security_log.csv")
    p.add_argument("--attack_time", type=float, default=200.0)
    p.add_argument("--malicious",   default="0",
                   help="ID droni maliciosi separati da virgola, es: 0,1")
    p.add_argument("--n_drones",    type=int, default=8)
    return p.parse_args()

# ─────────────────────────────────────────────────────────────
def load_csv(path):
    if not os.path.exists(path):
        for alt in ["../"+path, "../../"+path]:
            if os.path.exists(alt):
                path = alt
                break
        else:
            sys.exit(f"ERRORE: '{path}' non trovato.")
    df = pd.read_csv(path)
    print(f"[OK] CSV: {os.path.abspath(path)}  ({len(df)} righe)\n")
    return df

# ─────────────────────────────────────────────────────────────
# METRICA 1 – Detection Rate collettiva (almeno 1 osservatore onesto suona)
# ─────────────────────────────────────────────────────────────
def detection_rate(df, malicious_ids, attack_time, honest_ids):
    """
    Per ogni timestep post-attacco, guarda solo gli osservatori ONESTI.
    Se almeno uno di loro ha alarm=1 sul malicioso → timestep rilevato.
    Questo è l'unico modo corretto: escludiamo gli osservatori maliciosi
    che potrebbero far finta di rilevare i complici per sembrare onesti.
    """
    results = {}
    df_post = df[df["time"] >= attack_time]

    for mid in malicious_ids:
        # Solo le righe dove sender=attaccante E observer=onesto
        df_m = df_post[
            (df_post["sender_id"]   == mid) &
            (df_post["observer_id"].isin(honest_ids))
        ]
        if df_m.empty:
            results[mid] = None
            continue
        per_time = df_m.groupby("time")["alarm"].max()
        results[mid] = per_time.mean() * 100.0

    return results

# ─────────────────────────────────────────────────────────────
# METRICA 2 – Detection Latency (dal primo alarm di un osservatore onesto)
# ─────────────────────────────────────────────────────────────
def detection_latency(df, malicious_ids, attack_time, honest_ids):
    results = {}
    df_post = df[df["time"] >= attack_time]

    for mid in malicious_ids:
        df_m = df_post[
            (df_post["sender_id"]   == mid) &
            (df_post["observer_id"].isin(honest_ids)) &
            (df_post["alarm"] == 1)
        ]
        if df_m.empty:
            results[mid] = None
        else:
            results[mid] = df_m["time"].min() - attack_time

    return results

# ─────────────────────────────────────────────────────────────
# METRICA 3 – False Positive Rate pre-attacco (droni onesti)
# ─────────────────────────────────────────────────────────────
def false_positive_rate(df, malicious_ids, attack_time, honest_ids):
    df_pre = df[df["time"] < attack_time]
    rates = {}
    for hid in honest_ids:
        df_h = df_pre[
            (df_pre["sender_id"]   == hid) &
            (df_pre["observer_id"].isin(honest_ids))
        ]
        if df_h.empty:
            rates[hid] = 0.0
            continue
        per_time = df_h.groupby("time")["alarm"].max()
        rates[hid] = per_time.mean() * 100.0

    overall = np.mean(list(rates.values())) if rates else 0.0
    return overall, rates

# ─────────────────────────────────────────────────────────────
# METRICA 4 – EKF error droni onesti (effetto riga 632)
# ─────────────────────────────────────────────────────────────
def honest_ekf_error(df, malicious_ids, attack_time, honest_ids):
    df_h = df[df["sender_id"].isin(honest_ids)]
    pre  = df_h[df_h["time"] <  attack_time]["estimation_error"].mean()
    post = df_h[df_h["time"] >= attack_time]["estimation_error"].mean()
    return pre, post

# ─────────────────────────────────────────────────────────────
# METRICA 5 – Collasso del consenso (voti < soglia)
# ─────────────────────────────────────────────────────────────
def consensus_collapse(df, malicious_ids, attack_time, honest_ids):
    """
    Guarda solo le righe scritte da osservatori ONESTI per ogni attaccante.
    Se total_votes < threshold → il consenso non ha raggiunto la soglia
    in quel timestep → collasso parziale.
    """
    df_post = df[df["time"] >= attack_time]
    collapse_rates = {}

    for mid in malicious_ids:
        df_m = df_post[
            (df_post["sender_id"]   == mid) &
            (df_post["observer_id"].isin(honest_ids))
        ]
        if df_m.empty:
            collapse_rates[mid] = None
            continue
        g = df_m.groupby("time").agg({"total_votes": "max", "threshold": "max"})
        collapse_rates[mid] = (g["total_votes"] < g["threshold"]).mean() * 100.0

    return collapse_rates

# ─────────────────────────────────────────────────────────────
# METRICA 6 (NUOVA) – Copertura reciproca tra maliciosi
# ─────────────────────────────────────────────────────────────
def mutual_protection(df, malicious_ids, attack_time):
    """
    Questa è la metrica chiave per capire se i maliciosi si coprono.

    Per ogni attaccante A, guarda le righe dove:
      - sender_id  = A           (stiamo guardando A)
      - observer_id in malicious (un complice sta osservando A)
      - alarm = 0                (il complice NON suona l'allarme su A)

    Se questa percentuale è alta → i maliciosi si proteggono a vicenda
    non accusandosi → tentano di bloccare il quorum.

    Restituisce per ogni attaccante:
      - protection_rate: % di timestep in cui i complici NON lo accusano
      - n_complici_attivi: quanti complici diversi lo "proteggono"
    """
    if len(malicious_ids) < 2:
        return {}   # con 1 solo malicioso non ha senso

    df_post = df[df["time"] >= attack_time]
    results = {}

    for mid in malicious_ids:
        complici = [m for m in malicious_ids if m != mid]

        # Righe dove un complice osserva mid
        df_complice = df_post[
            (df_post["sender_id"]   == mid) &
            (df_post["observer_id"].isin(complici))
        ]
        if df_complice.empty:
            results[mid] = {"protection_rate": None, "n_complici": 0}
            continue

        # Quante volte il complice NON suona l'allarme su mid
        per_time = df_complice.groupby("time")["alarm"].min()  # min=0 se almeno uno tace
        protection_rate = (per_time == 0).mean() * 100.0

        # Quanti complici distinti hanno osservato mid almeno una volta
        n_complici = df_complice["observer_id"].nunique()

        results[mid] = {
            "protection_rate": protection_rate,
            "n_complici": n_complici
        }

    return results

# ─────────────────────────────────────────────────────────────
# STAMPA REPORT
# ─────────────────────────────────────────────────────────────
def print_report(malicious_ids, honest_ids, dr, latency, fpr,
                 ekf_pre, ekf_post, collapse, mutual):
    sep = "=" * 72
    print(sep)
    print("  SWARMRAFT RESILIENCE REPORT")
    print(sep)
    print(f"  Droni maliciosi : {malicious_ids}")
    print(f"  Droni onesti    : {honest_ids}")
    print(f"  FPR pre-attacco : {fpr:.2f}%")

    delta = ekf_post - ekf_pre
    tag = f"← ATTENZIONE riga-632! (+{delta:.3f}m)" if delta > 0.3 else f"← OK (Δ={delta:.3f}m)"
    print(f"  EKF onesti      : PRE={ekf_pre:.3f}m  POST={ekf_post:.3f}m  {tag}")

    print()
    print(f"  {'Drone':<7} {'Det.Rate':>10} {'Latency':>12} {'Collapse':>12} {'Mutual Prot.':>14}")
    print(f"  {'-'*7} {'-'*10} {'-'*12} {'-'*12} {'-'*14}")

    all_detected = True
    for mid in malicious_ids:
        dr_str  = f"{dr[mid]:.1f}%"       if dr[mid]      is not None else "N/A"
        lat_str = f"{latency[mid]:.2f}s"  if latency[mid] is not None else "MAI"
        col_str = f"{collapse[mid]:.1f}%" if collapse[mid] is not None else "N/A"

        if mid in mutual and mutual[mid]["protection_rate"] is not None:
            mp = mutual[mid]["protection_rate"]
            nc = mutual[mid]["n_complici"]
            mp_str = f"{mp:.1f}% ({nc} compl.)"
        else:
            mp_str = "—"

        if dr[mid] is None or dr[mid] < 50.0:
            all_detected = False

        print(f"  {mid:<7} {dr_str:>10} {lat_str:>12} {col_str:>12} {mp_str:>14}")

    print()
    # Verdetto
    if all_detected:
        print("  ✓ SISTEMA RESILIENTE: tutti gli attaccanti rilevati dagli onesti")
    else:
        print("  ✗ COLLASSO: almeno un attaccante NON rilevato dagli onesti")

    if mutual:
        high_prot = [m for m in malicious_ids
                     if m in mutual and mutual[m]["protection_rate"] is not None
                     and mutual[m]["protection_rate"] > 70.0]
        if high_prot:
            print(f"  ⚠ COPERTURA RECIPROCA ALTA per droni: {high_prot}")
            print("    → i complici non si accusano a vicenda (blocco quorum probabile)")

    print(sep)
    print()

# ─────────────────────────────────────────────────────────────
# GRAFICI (4 pannelli)
# ─────────────────────────────────────────────────────────────
def plot_results(df, malicious_ids, honest_ids, attack_time):
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle(
        f"SwarmRaft – Resilienza | Attaccanti: {malicious_ids}  "
        f"(attacco a t={attack_time}s)",
        fontsize=13, fontweight="bold"
    )
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.35)
    colors_mal = plt.cm.Reds(np.linspace(0.45, 0.9, max(len(malicious_ids), 1)))

    # ── 1. Voti vs soglia nel tempo ───────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.set_title("Voti di accusa (da osservatori ONESTI) vs Soglia quorum")

    for idx, mid in enumerate(malicious_ids):
        df_m = df[
            (df["sender_id"]   == mid) &
            (df["observer_id"].isin(honest_ids))
        ]
        if df_m.empty:
            continue
        g = df_m.groupby("time").agg({"total_votes": "max", "threshold": "max"})
        ax1.plot(g.index, g["total_votes"],
                 label=f"Voti vs Drone {mid}", color=colors_mal[idx], linewidth=2)
        if idx == 0:
            ax1.plot(g.index, g["threshold"], color="black",
                     linewidth=1.5, linestyle="--", label="Soglia quorum")

    ax1.axvline(attack_time, color="red", linewidth=1.5, linestyle=":",
                label=f"Inizio attacco")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel("Voti")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # ── 2. Detection Rate nel tempo (bin 10s, solo osservatori onesti) ────
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.set_title("Detection Rate nel tempo\n(solo osservatori onesti, bin 10s)")
    df_post = df[df["time"] >= attack_time]

    for idx, mid in enumerate(malicious_ids):
        df_m = df_post[
            (df_post["sender_id"]   == mid) &
            (df_post["observer_id"].isin(honest_ids))
        ].copy().sort_values("time")
        if df_m.empty:
            continue
        df_m["bin"] = (df_m["time"] // 10) * 10
        rolling = df_m.groupby("bin")["alarm"].mean() * 100
        ax2.plot(rolling.index, rolling.values,
                 label=f"Drone {mid}", color=colors_mal[idx],
                 linewidth=2, marker="o", markersize=4)

    ax2.axhline(50, color="orange", linestyle="--", linewidth=1, label="50%")
    ax2.axhline(90, color="green",  linestyle="--", linewidth=1, label="90%")
    ax2.set_ylim(0, 105)
    ax2.set_xlabel("Tempo [s]")
    ax2.set_ylabel("Detection Rate [%]")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # ── 3. Copertura reciproca: alarm dei complici su ogni attaccante ─────
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.set_title("Copertura reciproca\n(alarm dei COMPLICI su ogni attaccante)")

    if len(malicious_ids) >= 2:
        for idx, mid in enumerate(malicious_ids):
            complici = [m for m in malicious_ids if m != mid]
            df_m = df_post[
                (df_post["sender_id"]   == mid) &
                (df_post["observer_id"].isin(complici))
            ].copy().sort_values("time")
            if df_m.empty:
                continue
            df_m["bin"] = (df_m["time"] // 10) * 10
            # alarm=0 dai complici = protezione attiva
            protect = df_m.groupby("bin")["alarm"].apply(lambda x: (x == 0).mean() * 100)
            ax3.plot(protect.index, protect.values,
                     label=f"Prot. Drone {mid}", color=colors_mal[idx],
                     linewidth=2, marker="s", markersize=4)

        ax3.axhline(70, color="red", linestyle="--", linewidth=1,
                    label="70% soglia pericolo")
        ax3.set_ylim(0, 105)
        ax3.set_xlabel("Tempo [s]")
        ax3.set_ylabel("% timestep non accusati dai complici")
        ax3.legend(fontsize=9)
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, "N/A\n(serve ≥2 attaccanti)",
                 ha="center", va="center", transform=ax3.transAxes, fontsize=12)
        ax3.set_title("Copertura reciproca (N/A)")

    outfile = "resilience_plots.png"
    plt.savefig(outfile, dpi=150, bbox_inches="tight")
    print(f"[OK] Grafici salvati: {outfile}")
    plt.show()

# ─────────────────────────────────────────────────────────────
# SALVA CSV RIASSUNTIVO
# ─────────────────────────────────────────────────────────────
def save_summary_csv(malicious_ids, dr, latency, fpr, ekf_pre, ekf_post,
                     collapse, mutual, n_drones, outpath="resilience_report.csv"):
    rows = []
    for mid in malicious_ids:
        mp_rate = None
        mp_n    = 0
        if mid in mutual and mutual[mid]:
            mp_rate = mutual[mid].get("protection_rate")
            mp_n    = mutual[mid].get("n_complici", 0)

        rows.append({
            "n_drones":           n_drones,
            "n_attackers":        len(malicious_ids),
            "attacker_id":        mid,
            "detection_rate_pct": dr[mid],
            "latency_s":          latency[mid],
            "collapse_pct":       collapse[mid],
            "fpr_pct":            fpr,
            "ekf_error_pre_m":    ekf_pre,
            "ekf_error_post_m":   ekf_post,
            "ekf_delta_m":        ekf_post - ekf_pre,
            "mutual_prot_pct":    mp_rate,
            "n_complici_attivi":  mp_n,
        })

    out = pd.DataFrame(rows)
    header = not os.path.exists(outpath)
    out.to_csv(outpath, mode="a", index=False, header=header)
    print(f"[OK] resilience_report.csv aggiornato ({len(rows)} righe aggiunte)")

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────
def main():
    args = parse_args()
    malicious_ids = [int(x) for x in args.malicious.split(",")]
    honest_ids    = [i for i in range(args.n_drones) if i not in malicious_ids]

    df = load_csv(args.csv)

    dr      = detection_rate(df, malicious_ids, args.attack_time, honest_ids)
    latency = detection_latency(df, malicious_ids, args.attack_time, honest_ids)
    fpr, _  = false_positive_rate(df, malicious_ids, args.attack_time, honest_ids)
    ekf_pre, ekf_post = honest_ekf_error(df, malicious_ids, args.attack_time, honest_ids)
    collapse= consensus_collapse(df, malicious_ids, args.attack_time, honest_ids)
    mutual  = mutual_protection(df, malicious_ids, args.attack_time)

    print_report(malicious_ids, honest_ids, dr, latency, fpr,
                 ekf_pre, ekf_post, collapse, mutual)

    save_summary_csv(malicious_ids, dr, latency, fpr, ekf_pre, ekf_post,
                     collapse, mutual, args.n_drones)

    plot_results(df, malicious_ids, honest_ids, args.attack_time)


if __name__ == "__main__":
    main()