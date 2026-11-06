#!/usr/bin/env python3
"""Figure e tabelle finali per il paper (in inglese, formato ACM), dalle tabelle del rapporto (PAPER_NOTES 4.35).

Uso:  python3 tools/paper_figures.py --report runs/paper/report --out runs/paper/figures
Legge tab1_interni.csv, tab2_esterno.csv, tab3_misto.csv, tab4_scala.csv (scritte da tools/paper_report.py) e scrive
PDF vettoriali (font TrueType incorporati: ACM e HotCRP rifiutano i font Type 3) piu' un'anteprima PNG per ciascuno,
e le tabelle LaTeX tab_mixed.tex e tab_scale.tex. Le figure 1-2 (architettura, principio) sono schemi: bozze da
rifinire. Larghezze: colonna singola 3,33 in, doppia colonna 7 in (sigconf).

Criterio "il sistema regge" (figura della griglia): attaccanti rilevati >= 90% E falsi allarmi sugli onesti dopo
l'attacco <= 5% (mediane tra i seed), lo stesso del rapporto; "fragile" = regge in mediana ma almeno un seed crolla
(falsi allarmi > 50%, colonna `crollati` di tab1). Asse della griglia: frazione di attaccanti k/N.
"""
import argparse
import os

import numpy as np
import pandas as pd

SIZES = [5, 6, 8, 12, 20, 30, 50]
OKABE = ["#E69F00", "#56B4E9", "#009E73", "#CC79A7", "#0072B2", "#D55E00", "#000000"]
COL = dict(zip(SIZES, OKABE))
MARK = dict(zip(SIZES, ["o", "s", "^", "D", "v", "P", "X"]))
# Bianco e nero: ogni N ha forma E riempimento propri (pieno / vuoto alternati), le linee del limite i marcatori piccoli vuoti
HOLLOW = {5, 12, 30}


def mk(n, small=False):
    """Stile del marcatore di N: forma, colore, pieno o vuoto (leggibile anche stampato in bianco e nero)."""
    c = COL.get(n, "k")
    hollow = small or n in HOLLOW
    return dict(marker=MARK.get(n, "o"), color=c, mec=c, mfc="white" if hollow else c, ms=2.6 if small else 3.8,
                mew=0.8 if small else 1.0)
W1, W2 = 3.33, 7.0


def setup():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                         "ytick.labelsize": 7, "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2,
                         "lines.markersize": 3.5, "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight",
                         "savefig.pad_inches": 0.02,
                         "font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"]})   # incluso in matplotlib:
    # stesse figure su ogni macchina (senza, matplotlib sceglie il primo font di sistema disponibile)
    return plt


def holds(t):
    """Regge (mediane tra i seed); con un tab1 vecchio senza `rilevati_med` usa la media."""
    ril = t.rilevati_med if "rilevati_med" in t else t.rilevati
    return (ril >= 90) & (t.fa_med <= 5)


def r_axis(ax):
    """Asse della distanza dello spoofer: logaritmico con le tacche sulle distanze provate."""
    from matplotlib.ticker import FixedLocator, NullLocator, FixedFormatter
    ax.set_xscale("log")
    ax.xaxis.set_major_locator(FixedLocator([25, 50, 100, 200, 500, 1000]))
    ax.xaxis.set_major_formatter(FixedFormatter(["25", "50", "100", "200", "500", "1000"]))
    ax.xaxis.set_minor_locator(NullLocator())
    ax.set_xlabel("spoofer distance R [m]")


def log_y(ax, ticks):
    """Asse verticale logaritmico con tacche in numeri semplici (1, 2, 5, 10, ...)."""
    from matplotlib.ticker import FixedLocator, NullLocator, FixedFormatter
    ax.set_yscale("log")
    ax.yaxis.set_major_locator(FixedLocator(ticks))
    ax.yaxis.set_major_formatter(FixedFormatter([f"{x:g}" for x in ticks]))
    ax.yaxis.set_minor_locator(NullLocator())


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
    t["holds"] = holds(t)
    t["fragile"] = t.holds & (t.crollati > 0) if "crollati" in t else False
    full = t.groupby("N").seed.transform("max")
    t["complete"] = t.seed >= full
    Ns = [n for n in SIZES if n in set(t.N)]
    xi = {n: i for i, n in enumerate(Ns)}
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.4), sharey=True)
    # tre stati distinguibili anche in bianco e nero: pieno = regge, righe diagonali = regge ma qualche seed crolla,
    # quadretti = cede; quadrato vuoto senza texture = seed incompleti
    style = {"holds": dict(c="#009E73", ec="#009E73", hatch=None),
             "fragile": dict(c="white", ec="#009E73", hatch="//////"),
             "fails": dict(c="white", ec="#D55E00", hatch="xxxxxx")}
    plt.rcParams["hatch.linewidth"] = 0.6
    for ax, mode, title in zip(axs, ("coordinati", "indipendenti"), ("(a) coordinated insiders", "(b) independent insiders")):
        tm = t[t["mode"] == mode]
        for _, r in tm.iterrows():
            st = style["fragile" if r.fragile else ("holds" if r.holds else "fails")]
            face = st["c"] if r.complete else "white"
            ax.scatter(xi[r.N], r.k, marker="s", s=34, c=face, edgecolors=st["ec"], linewidths=0.9,
                       hatch=st["hatch"] if r.complete else None, zorder=3)
        xs = np.arange(len(Ns))
        ok = [i for i, n in enumerate(Ns) if n - 5 >= 1]   # k = N - 5 vale almeno 1 (asse logaritmico)
        ax.plot([xs[i] for i in ok], [Ns[i] - 5 for i in ok], "k--", lw=0.9, label="k = N - 5")
        ax.plot(xs, [n / 3 for n in Ns], ":", color="#555555", lw=1.0, label="k = N/3 (Byzantine voting)")
        ax.set_xticks(xs)
        ax.set_xticklabels([str(n) for n in Ns])
        ax.set_xlabel("swarm size N")
        ax.set_title(title, fontsize=8)
        log_y(ax, [1, 2, 3, 5, 10, 20])
        ax.set_ylim(0.75, 40)
    axs[0].set_ylabel("insiders k")
    from matplotlib.patches import Patch
    h = [Patch(facecolor="#009E73", edgecolor="#009E73", label="holds"),
         Patch(facecolor="white", edgecolor="#009E73", hatch="//////", label="holds, some seeds collapse"),
         Patch(facecolor="white", edgecolor="#D55E00", hatch="xxxxxx", label="fails")]
    if not t.complete.all():
        h.append(Patch(facecolor="white", edgecolor="#555555", label="incomplete seeds"))
    # legenda dentro il pannello (a), in alto a sinistra: li' non ci sono dati (con N <= 8 si arriva a k = 4)
    axs[0].legend(handles=h + axs[0].get_legend_handles_labels()[0], loc="upper left", fontsize=5.5,
                  framealpha=0.95, handlelength=1.6)
    return save(fig, out, "fig3_insider_grid", plt)


def fig_insider_latency(t1, out, plt):
    ok = t1[holds(t1)]
    t = ok[(ok.k == 1) & (ok["mode"] == "indipendenti")].sort_values("N")   # solo dove il sistema regge
    h = ok.groupby("N").latenza.agg(["min", "max"])
    fig, ax = plt.subplots(figsize=(W1, 1.9))
    ax.fill_between(h.index, h["min"], h["max"], facecolor="white", edgecolor="#56B4E9", hatch="////", lw=0.6,
                    label="all k where the system holds")
    ax.plot(t.N, t.latenza, "-", color="#0072B2", marker="o", mfc="#0072B2", label="one insider")
    ax.set_xlabel("swarm size N")
    ax.set_ylabel("detection latency [s]")
    ax.set_ylim(0, None)
    ax.legend(fontsize=6, loc="lower right", framealpha=0.95)
    return save(fig, out, "fig4_insider_latency", plt)


# ---------------------------------------------------------------- spoofer esterno
def fig_ext_lateral(t2, out, plt):
    t = t2[t2.livello == "B"]
    fig, ax = plt.subplots(figsize=(W1, 2.1))
    for n, g in t.groupby("N"):
        g = g.sort_values("R")
        ax.plot(g.R, g.laterale, label=f"N = {n}", **mk(n))
    r_axis(ax)
    log_y(ax, [1, 2, 5, 10, 20, 50])
    ax.set_ylabel("cross-range error [m]")
    ax.legend(ncol=2, fontsize=6, loc="upper left", framealpha=0.95)
    return save(fig, out, "fig5_spoofer_bearing", plt)


def fig_ext_range(t2, out, plt):
    t = t2[t2.livello == "B"]
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.1))
    for n, g in t.groupby("N"):
        g = g.sort_values("R")
        axs[0].plot(g.R, g.determinata, label=f"N = {n}", **mk(n))
        gg = g[g.determinata >= 50]
        axs[1].plot(gg.R, gg.dist_rel, label=f"N = {n}", **mk(n))
        axs[1].plot(g.R, g.ideale_dist_rel, ls=":", lw=0.8, **mk(n, small=True))   # limite: marcatori piccoli vuoti
    axs[0].set_ylabel("range resolved (error < R/2) [%]")
    axs[0].set_ylim(-34, 103)   # fascia sotto lo 0 per la legenda, dentro gli assi e lontana dai dati
    axs[0].set_yticks([0, 20, 40, 60, 80, 100])
    axs[1].set_ylabel("range error / R [%]")
    log_y(axs[1], [2, 5, 10, 20, 50, 100, 200])
    axs[1].text(0.02, 0.95, "solid: estimated (where resolved in >= 50%)\ndotted: geometric bound (true positions, true PPS)",
                transform=axs[1].transAxes, va="top", fontsize=6)
    for ax in axs:
        r_axis(ax)
    axs[0].axhline(-3, color="#999999", lw=0.5)
    # legenda dentro (a), nella fascia sotto lo 0: in alto copriva le curve con 30-50 droni
    axs[0].legend(ncol=4, fontsize=5.5, loc="lower center", framealpha=0.95, columnspacing=0.8, handletextpad=0.3)
    axs[0].set_title("(a)", fontsize=8)
    axs[1].set_title("(b)", fontsize=8)
    return save(fig, out, "fig6_spoofer_range", plt)


def fig_sensors(t2, out, plt):
    nd = sorted(t2[t2.livello == "D"].N.unique())
    if not nd:
        return None
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.1))
    for n in nd:
        for lev, ls, lab in (("B", "-", "good sensors"), ("D", "--", "poor sensors")):
            g = t2[(t2.N == n) & (t2.livello == lev)].sort_values("R")
            st = mk(n)
            if lev == "D":
                st["mfc"] = "white"   # sensori scarsi: tratteggio E marcatore vuoto (bianco e nero)
            axs[0].plot(g.R, g.dir_w20, ls=ls, label=f"N = {n}, {lab}", **st)
            axs[1].plot(g.R, g.determinata, ls=ls, label=f"N = {n}, {lab}", **st)
    axs[0].set_ylabel("bearing error [deg]")
    axs[1].set_ylabel("range resolved [%]")
    axs[1].set_ylim(-3, 103)
    for ax in axs:
        r_axis(ax)
    axs[0].set_ylim(0, 10.5)   # spazio in alto a destra per la legenda, dentro gli assi e senza dati
    axs[0].legend(fontsize=5.5, loc="upper right", framealpha=0.95)
    axs[0].set_title("(a)", fontsize=8)
    axs[1].set_title("(b)", fontsize=8)
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
