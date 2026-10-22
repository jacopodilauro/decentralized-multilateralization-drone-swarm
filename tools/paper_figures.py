#!/usr/bin/env python3
"""Figure e tabelle finali per il paper (in inglese, formato ACM), dalle tabelle del rapporto (PAPER_NOTES 4.35).

Uso:  python3 tools/paper_figures.py --report runs/paper/report --out runs/paper/figures
Legge tab1_interni.csv, tab2_esterno.csv, tab3_misto.csv, tab4_scala.csv (scritte da tools/paper_report.py) e scrive
PDF vettoriali (font TrueType incorporati: ACM e HotCRP rifiutano i font Type 3) piu' un'anteprima PNG per ciascuno,
e le tabelle LaTeX tab_mixed.tex e tab_scale.tex. Le figure 1-2 (architettura, principio) sono schemi: bozze da
rifinire. Larghezze: colonna singola 3,33 in, doppia colonna 7 in (sigconf).

Criterio "il sistema regge" (figura della griglia): attaccanti rilevati >= 90% E falsi allarmi sugli onesti dopo
l'attacco <= 5% (mediana tra i seed), lo stesso del rapporto.
"""
import argparse
import os

import numpy as np
import pandas as pd

SIZES = [5, 6, 8, 12, 20, 30, 50]
OKABE = ["#E69F00", "#56B4E9", "#009E73", "#CC79A7", "#0072B2", "#D55E00", "#000000"]
COL = dict(zip(SIZES, OKABE))
MARK = dict(zip(SIZES, ["o", "s", "^", "D", "v", "P", "X"]))
W1, W2 = 3.33, 7.0


def setup():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                         "ytick.labelsize": 7, "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2,
                         "lines.markersize": 3.5, "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight",
                         "savefig.pad_inches": 0.02})
    return plt


def save(fig, out, name, plt):
    fig.savefig(os.path.join(out, name + ".pdf"))
    fig.savefig(os.path.join(out, name + ".png"), dpi=200)
    plt.close(fig)
    return name


# ---------------------------------------------------------------- figure di progetto (bozze)
def fig_architecture(out, plt):
    from matplotlib.patches import FancyBboxPatch
    fig, ax = plt.subplots(figsize=(W2, 2.7))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 41)
    ax.axis("off")
    boxes = {   # x, y, larghezza, altezza, testo; tre righe: ranging, GNSS e decisione, sensori di bordo
        "uwb": (1, 30, 18, 9, "UWB TDMA broadcast\n(DS-TWR timestamps)"),
        "rng": (23, 30, 15, 9, "Inter-drone\nranges"),
        "ekf": (42, 30, 17, 9, "Cooperative EKF\n(one per neighbour)"),
        "gnss": (23, 16, 15, 9, "GNSS receiver\n(position, PPS)"),
        "test": (42, 16, 17, 9, "GNSS vs UWB\nconsistency test"),
        "vote": (63, 16, 12, 9, "Quorum vote\n(2/3)"),
        "ins": (79, 26, 20, 9, "Insider:\nexcluded from anchors"),
        "cap": (79, 4, 20, 11, "Capture:\nUWB-only frame,\nPPS TDoA ->\nspoofer bearing, range"),
        "imu": (42, 2, 17, 9, "On-board velocity\n(IMU, flow, compass)"),
    }
    for k, (x, y, w, h, t) in boxes.items():
        fc = "#E8F1FA" if k not in ("ins", "cap") else "#FDEBD3"
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3,rounding_size=1.2", fc=fc, ec="#333333", lw=0.8))
        ax.text(x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=6)

    def arrow(a, b, text=None, dx=0.0, dy=1.0):
        ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="->", lw=0.8, color="#333333"))
        if text:
            ax.text((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy, text, ha="center", fontsize=5.5, color="#555555")
    arrow((19.6, 34.5), (22.4, 34.5))                     # UWB -> distanze
    arrow((38.6, 34.5), (41.4, 34.5))                     # distanze -> EKF
    arrow((50.5, 29.4), (50.5, 25.6), "estimated positions", dx=9.0, dy=-0.8)
    arrow((38.6, 20.5), (41.4, 20.5))                     # GNSS -> test
    arrow((59.6, 20.5), (62.4, 20.5))                     # test -> voto
    arrow((75.6, 22.5), (78.4, 29.0))                     # voto -> interno
    arrow((75.6, 18.5), (78.4, 12.0))                     # voto -> cattura
    arrow((59.6, 6.5), (78.4, 8.0), "used only under capture", dy=1.2)
    return save(fig, out, "fig1_architecture", plt)


def fig_principle(out, plt):
    s = np.array([0.0, 0.0])
    ang = np.radians(np.arange(7) * 360.0 / 7 + 10.0)
    P = np.array([60.0, 20.0]) + 14.0 * np.c_[np.cos(ang), np.sin(ang)]   # formazione regolare
    fig, ax = plt.subplots(figsize=(W1, 2.3))
    X, Y = np.meshgrid(np.linspace(-15, 90, 500), np.linspace(-25, 50, 400))
    for i, j, c in ((0, 3, "#0072B2"), (1, 5, "#D55E00")):
        d = np.hypot(X - P[i, 0], Y - P[i, 1]) - np.hypot(X - P[j, 0], Y - P[j, 1])
        dd = np.hypot(*(P[i] - s)) - np.hypot(*(P[j] - s))
        ax.contour(X, Y, d - dd, levels=[0], colors=[c], linewidths=1.0)
    for p in P:
        ax.plot([s[0], p[0]], [s[1], p[1]], color="#999999", lw=0.5, ls=":")
    ax.plot(P[:, 0], P[:, 1], "o", color="#0072B2", ms=4, label="captured drones")
    for i, j, c in ((0, 3, "#0072B2"), (1, 5, "#D55E00")):
        for q, lab in ((i, "i"), (j, "j")):
            ax.annotate(lab, P[q], textcoords="offset points", xytext=(3, 3), fontsize=6, color=c)
    ax.plot(*s, "*", color="#D55E00", ms=10, label="spoofer antenna")
    ax.text(2, -9, r"$\mathrm{PPS}_i-\mathrm{PPS}_j=(\|p_i-s\|-\|p_j-s\|)/c$", fontsize=7)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="upper left", fontsize=6)
    ax.grid(False)
    return save(fig, out, "fig2_principle", plt)


# ---------------------------------------------------------------- attaccanti interni
def fig_insider_grid(t1, out, plt):
    t = t1.copy()
    t["holds"] = (t.rilevati >= 90) & (t.fa_med <= 5)
    full = t.groupby("N").seed.transform("max")
    t["complete"] = t.seed >= full
    Ns = [n for n in SIZES if n in set(t.N)]
    xi = {n: i for i, n in enumerate(Ns)}
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.4), sharey=True)
    for ax, mode, title in zip(axs, ("coordinati", "indipendenti"), ("coordinated insiders", "independent insiders")):
        tm = t[t["mode"] == mode]
        for _, r in tm.iterrows():
            c = "#009E73" if r.holds else "#D55E00"
            ax.scatter(xi[r.N], r.k, marker="s", s=70, c=c if r.complete else "white", edgecolors=c, linewidths=1.2)
        xs = np.arange(len(Ns))
        ax.plot(xs, [n - 5 for n in Ns], "k--", lw=0.9, label="k = N - 5")
        ax.plot(xs, [n / 3 for n in Ns], ":", color="#555555", lw=0.9, label="k = N/3 (Byzantine voting)")
        ax.set_xticks(xs)
        ax.set_xticklabels([str(n) for n in Ns])
        ax.set_xlabel("swarm size N")
        ax.set_title(title, fontsize=8)
        ax.set_ylim(0, max(t.k.max() + 1.5, 5))
    axs[0].set_ylabel("insiders k")
    from matplotlib.lines import Line2D
    h = [Line2D([], [], marker="s", ls="", mfc="#009E73", mec="#009E73", ms=6, label="holds"),
         Line2D([], [], marker="s", ls="", mfc="#D55E00", mec="#D55E00", ms=6, label="fails"),
         Line2D([], [], marker="s", ls="", mfc="white", mec="#555555", ms=6, label="incomplete seeds")]
    axs[1].legend(handles=h + axs[1].get_legend_handles_labels()[0], loc="upper left", fontsize=6)
    return save(fig, out, "fig3_insider_grid", plt)


def fig_insider_latency(t1, out, plt):
    t = t1[(t1.k == 1) & (t1["mode"] == "indipendenti")].sort_values("N")
    h = t1[(t1.rilevati >= 90) & (t1.fa_med <= 5)].groupby("N").latenza.agg(["min", "max"])
    fig, ax = plt.subplots(figsize=(W1, 1.9))
    ax.fill_between(h.index, h["min"], h["max"], color="#56B4E9", alpha=0.3, lw=0, label="all k where the system holds")
    ax.plot(t.N, t.latenza, "o-", color="#0072B2", label="one insider")
    ax.set_xlabel("swarm size N")
    ax.set_ylabel("detection latency [s]")
    ax.set_ylim(0, None)
    ax.legend(fontsize=6)
    return save(fig, out, "fig4_insider_latency", plt)


# ---------------------------------------------------------------- spoofer esterno
def fig_ext_lateral(t2, out, plt):
    t = t2[t2.livello == "B"]
    fig, ax = plt.subplots(figsize=(W1, 2.1))
    for n, g in t.groupby("N"):
        g = g.sort_values("R")
        ax.plot(g.R, g.laterale, marker=MARK.get(n, "o"), color=COL.get(n, "k"), label=f"N = {n}")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("spoofer distance R [m]")
    ax.set_ylabel("cross-range error [m]")
    ax.legend(ncol=2, fontsize=6)
    return save(fig, out, "fig5_spoofer_bearing", plt)


def fig_ext_range(t2, out, plt):
    t = t2[t2.livello == "B"]
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.1))
    for n, g in t.groupby("N"):
        g = g.sort_values("R")
        axs[0].plot(g.R, g.determinata, marker=MARK.get(n, "o"), color=COL.get(n, "k"), label=f"N = {n}")
        gg = g[g.determinata >= 50]
        axs[1].plot(gg.R, gg.dist_rel, marker=MARK.get(n, "o"), color=COL.get(n, "k"), label=f"N = {n}")
        axs[1].plot(g.R, g.ideale_dist_rel, ls=":", color=COL.get(n, "k"), lw=0.9)
    axs[0].set_ylabel("range resolved (error < R/2) [%]")
    axs[0].set_ylim(-3, 103)
    axs[1].set_ylabel("range error / R [%]")
    axs[1].set_yscale("log")
    axs[1].text(0.02, 0.95, "solid: estimated (where resolved in >= 50%)\ndotted: geometric bound (true positions, true PPS)",
                transform=axs[1].transAxes, va="top", fontsize=6)
    for ax in axs:
        ax.set_xscale("log")
        ax.set_xlabel("spoofer distance R [m]")
    axs[0].legend(ncol=2, fontsize=6)
    return save(fig, out, "fig6_spoofer_range", plt)


def fig_sensors(t2, out, plt):
    nd = sorted(t2[t2.livello == "D"].N.unique())
    if not nd:
        return None
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.1))
    for n in nd:
        for lev, ls, lab in (("B", "-", "good sensors"), ("D", "--", "poor sensors")):
            g = t2[(t2.N == n) & (t2.livello == lev)].sort_values("R")
            axs[0].plot(g.R, g.dir_w20, ls=ls, marker=MARK.get(n, "o"), color=COL.get(n, "k"), label=f"N = {n}, {lab}")
            axs[1].plot(g.R, g.determinata, ls=ls, marker=MARK.get(n, "o"), color=COL.get(n, "k"), label=f"N = {n}, {lab}")
    axs[0].set_ylabel("bearing error [deg]")
    axs[1].set_ylabel("range resolved [%]")
    axs[1].set_ylim(-3, 103)
    for ax in axs:
        ax.set_xscale("log")
        ax.set_xlabel("spoofer distance R [m]")
    axs[1].legend(fontsize=6)
    return save(fig, out, "fig7_onboard_sensors", plt)


# ---------------------------------------------------------------- tabelle LaTeX
def tex_mixed(t3, out):
    lines = [r"\begin{tabular}{rrrrrrrr}", r"\toprule",
             r"$N$ & $k$ & $R$ [m] & insiders & capture & capture & bearing [deg] & range \\",
             r" & & & detected & consensus & latency [s] & (no insiders) & resolved [\%] \\", r"\midrule"]
    for _, r in t3.sort_values(["N", "k", "R"]).iterrows():
        lines.append(f"{r.N:.0f} & {r.k:.0f} & {r.R:.0f} & {r.rilevati:.0f}\\% & {r.consenso:.0f}\\% & "
                     f"{r.lat_cattura:.2f} & {r.dir_w20:.1f} ({r.dir_w20_solo:.1f}) & {r.determinata:.0f} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}"]
    open(os.path.join(out, "tab_mixed.tex"), "w").write("\n".join(lines) + "\n")


def tex_scale(t4, out):
    lines = [r"\begin{tabular}{rrrr}", r"\toprule", r"$N$ & largest packet [B] & TDMA frame [ms] & shell radius [m] \\",
             r"\midrule"]
    for _, r in t4.sort_values("N").iterrows():
        lines.append(f"{r.N:.0f} & {r.pacchetto:.0f} & {r.frame:.0f} & {r.raggio:.1f} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}"]
    open(os.path.join(out, "tab_scale.tex"), "w").write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", default="runs/paper/report")
    ap.add_argument("--out", default="runs/paper/figures")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    plt = setup()
    made = [fig_architecture(a.out, plt), fig_principle(a.out, plt)]
    rd = lambda n: pd.read_csv(os.path.join(a.report, n)) if os.path.exists(os.path.join(a.report, n)) else None  # noqa: E731
    t1, t2, t3, t4 = rd("tab1_interni.csv"), rd("tab2_esterno.csv"), rd("tab3_misto.csv"), rd("tab4_scala.csv")
    if t1 is not None:
        made += [fig_insider_grid(t1, a.out, plt), fig_insider_latency(t1, a.out, plt)]
    if t2 is not None:
        made += [fig_ext_lateral(t2, a.out, plt), fig_ext_range(t2, a.out, plt), fig_sensors(t2, a.out, plt)]
    if t3 is not None:
        tex_mixed(t3, a.out)
        made.append("tab_mixed.tex")
    if t4 is not None:
        tex_scale(t4, a.out)
        made.append("tab_scale.tex")
    print(">>> figure e tabelle in " + a.out + ": " + ", ".join(m for m in made if m))


if __name__ == "__main__":
    main()
