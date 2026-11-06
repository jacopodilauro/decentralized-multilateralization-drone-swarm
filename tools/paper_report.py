#!/usr/bin/env python3
"""Rapporto della campagna per il paper: tabelle, grafici e spiegazioni (PAPER_NOTES 4.33).

Uso:  python3 tools/paper_report.py --out runs/paper
Legge runs/paper/<interni|esterno|misto>/risultati.csv (scritti da tools/paper_campaign.py, anche a campagna non
finita) e scrive runs/paper/report/: report.md (da aprire con un visualizzatore Markdown), le figure fig*.png e le
tabelle tab*.csv. Le frasi con i numeri sono calcolate dai dati; le parti descrittive spiegano scenario, sensori e
metriche. E' un documento di lavoro, non testo per il paper.

Convenzioni per la confrontabilita': stesso colore per lo stesso N in tutte le figure; stessa scala sui pannelli
che mostrano la stessa grandezza; mediana tra i seed con barre dal 25 al 75 percentile; percentuali (frazioni di
attaccanti rilevati, distanze determinate) come media tra i seed.
"""
import argparse
import math
import os

import numpy as np
import pandas as pd

SIZES = [5, 6, 8, 12, 20, 30, 50]
COLORS = {n: f"C{i}" for i, n in enumerate(SIZES)}
MODES = ("coordinati", "indipendenti")


def holds(t):
    """Il sistema regge: attaccanti rilevati >= 90% e falsi allarmi sugli onesti <= 5%, mediane tra i seed (la media dei
    rilevati e' trascinata giu' da pochi seed crollati, PAPER_NOTES 4.44)."""
    return (t.rilevati_med >= 90) & (t.fa_med <= 5)


def load(out, exp):
    p = os.path.join(out, exp, "risultati.csv")
    if not os.path.exists(p):
        return None
    d = pd.read_csv(p)
    if "ok" in d.columns:
        d = d[d.ok == 1].copy()
    return d if len(d) else None


def q(x, f):
    x = pd.Series(x).dropna()
    return float(np.percentile(x, f)) if len(x) else np.nan


def fmt(v, nd=1):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}"


def md_table(df, cols, heads, nd):
    """Tabella Markdown senza dipendenze esterne."""
    lines = ["| " + " | ".join(heads) + " |", "|" + "---|" * len(heads)]
    for _, r in df.iterrows():
        cells = []
        for c, n in zip(cols, nd):
            v = r[c]
            cells.append(str(v) if n is None else fmt(v, n))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def med_iqr(d, by, col):
    """Mediana e 25/75 percentile di col per gruppo (compatibile con tutte le versioni di pandas)."""
    g = d.groupby(by)[col]
    return pd.DataFrame({"med": g.median(), "q1": g.quantile(0.25), "q3": g.quantile(0.75)}).dropna(subset=["med"])


def errbar(ax, x, g, color, label, marker="o"):
    y = g["med"].values
    lo = y - g["q1"].values
    hi = g["q3"].values - y
    ax.errorbar(x, y, yerr=[np.clip(lo, 0, None), np.clip(hi, 0, None)], color=color, marker=marker, ms=4,
                capsize=2, lw=1.3, label=label)


def plt_setup():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 130})
    return plt


# ------------------------------------------------------------------------------------------------------------------
# Esperimento 1: attaccanti interni
# ------------------------------------------------------------------------------------------------------------------
def report_interni(d, rep, plt):
    d = d.copy()
    d["kN"] = d.k / d.N
    fa = "int.fa_collettivi_onesti_da_onesti_post_pct"
    er = "int.errore_ekf_onesti_post_mediana_m"
    pre = "int.fa_collettivi_pre_attacco_pct"
    rows = []
    for (n, k, mode), g in d.groupby(["N", "k", "mode"]):
        rows.append(dict(N=n, k=k, mode=mode, kN=k / n, seed=len(g),
                         rilevati=g.attaccanti_rilevati_pct.mean(),
                         rilevati_med=g.attaccanti_rilevati_pct.median(),
                         crollati=int((g[fa] > 50).sum()),
                         latenza=g.latenza_rilevamento_s.median(),
                         fa_med=g[fa].median(), fa_q3=q(g[fa], 75),
                         err_med=g[er].median(), err_q3=q(g[er], 75),
                         fa_pre=g[pre].median() if pre in g else np.nan))
    t = pd.DataFrame(rows).sort_values(["mode", "N", "k"])
    t.to_csv(os.path.join(rep, "tab1_interni.csv"), index=False)

    # Figure 1-3: rilevati, falsi allarmi, errore onesti contro k/N; un pannello per modalita', stessa scala
    figs = []
    for fid, col, lab, ylim, logy in (("fig1_interni_rilevati.png", "rilevati", "attaccanti rilevati [%]", (-3, 105), False),
                                      ("fig2_interni_falsi_allarmi.png", "fa_med", "falsi allarmi sugli onesti dopo l'attacco [%]", None, False),
                                      ("fig3_interni_errore_onesti.png", "err_med", "errore di stima degli onesti, mediana [m]", None, True)):
        fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True, sharex=True)
        handles = {}
        for ax, mode in zip(axs, MODES):
            tm = t[t["mode"] == mode]
            for n, g in tm.groupby("N"):
                h, = ax.plot(g.kN, g[col], "o-", color=COLORS.get(n, "k"), ms=4, label=f"N = {n}")
                handles[n] = h
            ax.axvline(1 / 3, color="k", ls="--", lw=0.8)
            ax.set_title(f"attaccanti {mode}")
            ax.set_xlabel("frazione di attaccanti k/N")
            if logy:
                ax.set_yscale("log")
        axs[0].set_ylabel(lab)
        axs[0].set_xlim(0, max(0.5, float(t.kN.max()) + 0.05))   # stessa scala in tutte le figure 1-3
        if ylim:
            axs[0].set_ylim(*ylim)
        for ax in axs:
            ax.text(1 / 3, ax.get_ylim()[1], " k/N = 1/3", va="top", fontsize=7)
        if handles:
            axs[1].legend([handles[n] for n in sorted(handles)], [f"N = {n}" for n in sorted(handles)], fontsize=7, ncol=2)
        fig.tight_layout()
        fig.savefig(os.path.join(rep, fid))
        plt.close(fig)
        figs.append(fid)

    # Figura 4: latenza di rilevamento con un solo attaccante, contro N
    fig, ax = plt.subplots(figsize=(5, 3.2))
    for mode, mk in zip(MODES, ("o", "s")):
        g = med_iqr(d[(d.k == 1) & (d["mode"] == mode)], "N", "latenza_rilevamento_s")
        if len(g):
            errbar(ax, g.index.values, g, "C0" if mode == "coordinati" else "C3", mode, mk)
    ax.set_xlabel("droni N")
    ax.set_ylabel("latenza di rilevamento [s]")
    ax.set_title("un attaccante interno")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "fig4_interni_latenza.png"))
    plt.close(fig)
    figs.append("fig4_interni_latenza.png")

    # Osservazioni calcolate (solo sulle configurazioni con tutti i seed: una campagna interrotta lascia configurazioni
    # con meno seed, che non vanno usate per le conclusioni)
    obs = []
    full = t.groupby("N").seed.transform("max")
    inc = t[t.seed < full]
    t_c = t[t.seed >= full]
    if len(inc):
        obs.append("- **Attenzione: configurazioni con seed incompleti (escluse dalle frasi seguenti):** " + "; ".join(
            f"N = {r.N}, k = {r.k}, {r['mode']} ({r.seed} seed)" for _, r in inc.iterrows()) + ".")
    for mode in MODES:
        tm = t_c[t_c["mode"] == mode]
        parts = []
        for n, g in tm.groupby("N"):
            g = g.sort_values("k")
            ok = g[holds(g)]
            if not len(ok):
                parts.append(f"N = {n}: per nessun k")
                continue
            r = ok.iloc[-1]
            txt = f"N = {n}: fino a k/N = {r.kN:.2f}"
            if r.crollati:
                txt += f" (ma {r.crollati}/{r.seed} seed crollati)"
            nxt = g[g.k > r.k]
            if len(nxt) and nxt.iloc[0].k - r.k > 1:   # vuoto nella griglia: il limite vero e' tra i due k provati
                txt += f" (k da {r.k + 1} a {nxt.iloc[0].k - 1} non provati: il limite e' tra k/N = {r.kN:.2f} e {nxt.iloc[0].kN:.2f})"
            parts.append(txt)
        obs.append(f"- Attaccanti **{mode}**: il sistema funziona (attaccanti rilevati >= 90% E falsi allarmi sugli "
                   f"onesti <= 5%, mediane tra i seed; crollato = seed con falsi allarmi > 50%) " + "; ".join(parts) +
                   ". Rilevare tutti gli attaccanti non basta se si accusano anche gli onesti: le due condizioni vanno "
                   "lette insieme (Figure 1 e 2).")
    one = t[t.k == 1]
    if len(one):
        bad = one[one.fa_med > 5]
        good = one[one.fa_med <= 5]
        if len(good):
            obs.append(f"- Con **un solo attaccante** i falsi allarmi sugli onesti restano al massimo "
                       f"{good.fa_med.max():.2f}% (mediana tra i seed) per N = {', '.join(str(x) for x in sorted(good.N.unique()))}.")
        if len(bad):
            rng = (f"{bad.fa_med.min():.1f}%" if bad.fa_med.min() == bad.fa_med.max()
                   else f"{bad.fa_med.min():.1f}-{bad.fa_med.max():.1f}%")
            obs.append(f"- Con un solo attaccante e N = {', '.join(str(x) for x in sorted(bad.N.unique()))} i falsi "
                       f"allarmi sugli onesti salgono a {rng}: il filtro aggiorna "
                       f"un drone solo con almeno 4 misure, e togliendo l'attaccante come ancora ne restano meno di 4 "
                       f"(regola N >= k + 5, verificata con N = 5 contro N = 6).")
    lat1 = d[d.k == 1].latenza_rilevamento_s
    if lat1.notna().any():
        obs.append(f"- Latenza di rilevamento con un attaccante: mediana {lat1.median():.2f} s "
                   f"(25-75 percentile {q(lat1, 25):.2f}-{q(lat1, 75):.2f} s) su tutte le dimensioni.")

    txt = ["## 2. Esperimento 1: attaccanti interni", "",
           "**Cosa succede.** A t = 100 s, k droni dello sciame iniziano a dichiarare una posizione GPS falsa, spostata di "
           "15 m con una rampa di 10 s. *Coordinati*: tutti spostati verso nord (si coprono a vicenda, perche' tra loro "
           "le distanze GPS restano coerenti). *Indipendenti*: ognuno in una direzione diversa. Gli attaccanti sono "
           "distribuiti uniformemente sul guscio; il drone 0 al centro e' sempre onesto.",
           "", "**Come lo misuriamo** (stesse definizioni di `tools/metrics.py` usate in tutto il lavoro):",
           "- *attaccante rilevato*: almeno meta' dei droni che lo osservano accende l'allarme collettivo (voto a "
           "quorum 2/3) dopo l'attacco; *attaccanti rilevati [%]* = media tra i seed della frazione rilevata;",
           "- *latenza*: dall'inizio dell'attacco al primo allarme collettivo, mediana tra gli osservatori;",
           "- *falsi allarmi sugli onesti*: frazione delle osservazioni onesto -> onesto con allarme collettivo dopo "
           "l'attacco (esclusi gli attaccanti anche come osservatori, perche' vedono gli altri fuori posto);",
           "- *errore di stima degli onesti*: distanza tra posizione stimata e vera, osservazioni onesto -> onesto.",
           "", "**Perche' 1/3.** Un allarme collettivo richiede i voti di 2/3 dei droni attivi. Se piu' di 1/3 sono "
           "attaccanti, gli onesti da soli non raggiungono il quorum: la teoria prevede che il rilevamento crolli. "
           "La linea tratteggiata nelle figure e' k/N = 1/3.", "", "**Osservazioni (calcolate dai dati).**", *obs, "",
           "![rilevati](fig1_interni_rilevati.png)", "",
           "*Figura 1.* Attaccanti rilevati contro la frazione di attaccanti, per dimensione dello sciame.", "",
           "![falsi allarmi](fig2_interni_falsi_allarmi.png)", "",
           "*Figura 2.* Falsi allarmi sui droni onesti dopo l'attacco (mediana tra i seed).", "",
           "![errore onesti](fig3_interni_errore_onesti.png)", "",
           "*Figura 3.* Errore di stima dei droni onesti (scala logaritmica).", "",
           "![latenza](fig4_interni_latenza.png)", "",
           "*Figura 4.* Latenza di rilevamento con un attaccante (mediana, barre 25-75 percentile).", "",
           "*Tabella 1.* Per configurazione: seed, attaccanti rilevati (media e mediana %), seed crollati (falsi allarmi "
           "sugli onesti > 50%), latenza mediana, falsi allarmi sugli onesti (mediana e 75 percentile), errore degli "
           "onesti (mediana e 75 percentile), falsi allarmi prima dell'attacco.", "",
           md_table(t, ["N", "k", "mode", "kN", "seed", "rilevati", "rilevati_med", "crollati", "latenza", "fa_med", "fa_q3",
                        "err_med", "err_q3", "fa_pre"],
                    ["N", "k", "modalita'", "k/N", "seed", "rilevati %", "rilevati med %", "crollati", "latenza s",
                     "FA onesti %", "FA p75 %", "errore m", "errore p75 m", "FA pre %"],
                    [None, None, None, 2, None, 0, 0, None, 2, 2, 2, 2, 2, 2]), ""]
    return txt


# ------------------------------------------------------------------------------------------------------------------
# Esperimento 2: spoofer esterno
# ------------------------------------------------------------------------------------------------------------------
def ext_table(d):
    d = d.copy()
    d["laterale_m"] = d.R * np.tan(np.radians(d["loc.reale_azimut_fusione_W20_gradi"]))
    d["dist_rel_pct"] = 100.0 * d["loc.reale_distanza_mediana_m"] / d.R
    d["consenso_pct"] = 100.0 * d.droni_consenso_cattura / d.N
    rows = []
    for (n, R, lev), g in d.groupby(["N", "R", "level"]):
        rows.append(dict(N=n, R=R, livello=lev, seed=len(g), consenso=g.consenso_pct.mean(),
                         lat_cattura=g["est.consenso_cattura_latenza_mediana_s"].median(),
                         dir_tutto=g["loc.reale_azimut_fusione_gradi"].median(),
                         dir_w20=g["loc.reale_azimut_fusione_W20_gradi"].median(),
                         dir_w20_q3=q(g["loc.reale_azimut_fusione_W20_gradi"], 75),
                         laterale=g.laterale_m.median(),
                         dist_rel=g.dist_rel_pct.median(),
                         determinata=g["loc.reale_distanza_determinata_pct"].mean(),
                         ideale_dist_rel=(100.0 * g["loc.ideale_distanza_mediana_m"] / g.R).median(),
                         rotazione=g["loc.rotazione_mediana_gradi"].median(),
                         forma=g["loc.forma_m"].median(), apertura=g["loc.apertura_m"].median()))
    return d, pd.DataFrame(rows).sort_values(["livello", "N", "R"])


def report_esterno(d, rep, plt):
    d, t = ext_table(d)
    t.to_csv(os.path.join(rep, "tab2_esterno.csv"), index=False)
    tb = t[t.livello == "B"]

    # Figura 5: direzione (finestra iniziale) e errore laterale contro R
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6))
    for n, g in tb.groupby("N"):
        axs[0].plot(g.R, g.dir_w20, "o-", color=COLORS.get(n, "k"), ms=4, label=f"N = {n}")
        axs[1].plot(g.R, g.laterale, "o-", color=COLORS.get(n, "k"), ms=4, label=f"N = {n}")
    axs[0].set_ylabel("errore di direzione, finestra 20 s [gradi]")
    axs[1].set_ylabel("errore laterale = R tan(errore) [m]")
    axs[1].set_yscale("log")
    for ax in axs:
        ax.set_xscale("log")
        ax.set_xlabel("distanza dello spoofer R [m]")
    axs[1].legend(fontsize=7, ncol=2)
    fig.suptitle("spoofer esterno, sensori di bordo livello B", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "fig5_esterno_direzione.png"))
    plt.close(fig)

    # Figura 6: distanza determinata e errore relativo contro R
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6))
    for n, g in tb.groupby("N"):
        axs[0].plot(g.R, g.determinata, "o-", color=COLORS.get(n, "k"), ms=4, label=f"N = {n}")
        gg = g[g.determinata >= 50]
        axs[1].plot(gg.R, gg.dist_rel, "o-", color=COLORS.get(n, "k"), ms=4, label=f"N = {n}")
        axs[1].plot(g.R, g.ideale_dist_rel, "x:", color=COLORS.get(n, "k"), ms=4, alpha=0.6)
    axs[0].set_ylabel("distanza determinata (errore < R/2) [%]")
    axs[0].set_ylim(-3, 105)
    axs[1].set_ylabel("errore di distanza / R [%] (x: ideale)")
    axs[1].set_yscale("log")
    for ax in axs:
        ax.set_xscale("log")
        ax.set_xlabel("distanza dello spoofer R [m]")
    axs[0].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "fig6_esterno_distanza.png"))
    plt.close(fig)
    figs = ["fig5_esterno_direzione.png", "fig6_esterno_distanza.png"]

    # Figura 7: livello B contro D (sensori di bordo buoni contro scarsi)
    td = t[t.N.isin(t[t.livello == "D"].N.unique())]
    if (t.livello == "D").any():
        fig, axs = plt.subplots(1, 2, figsize=(10, 3.6))
        for (n, lev), g in td.groupby(["N", "livello"]):
            ls = "o-" if lev == "B" else "s--"
            axs[0].plot(g.R, g.dir_w20, ls, color=COLORS.get(n, "k"), ms=4, label=f"N = {n}, livello {lev}")
            axs[1].plot(g.R, g.determinata, ls, color=COLORS.get(n, "k"), ms=4, label=f"N = {n}, livello {lev}")
        axs[0].set_ylabel("errore di direzione, finestra 20 s [gradi]")
        axs[1].set_ylabel("distanza determinata [%]")
        axs[1].set_ylim(-3, 105)
        for ax in axs:
            ax.set_xscale("log")
            ax.set_xlabel("distanza dello spoofer R [m]")
        axs[1].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(rep, "fig7_esterno_sensori.png"))
        plt.close(fig)
        figs.append("fig7_esterno_sensori.png")

    obs = []
    if len(tb):
        c = tb.consenso
        obs.append(f"- Cattura riconosciuta dallo sciame: in media {c.min():.0f}-{c.max():.0f}% dei droni raggiungono il "
                   f"consenso, latenza mediana {tb.lat_cattura.min():.2f}-{tb.lat_cattura.max():.2f} s.")
        for R, g in tb.groupby("R"):
            obs.append(f"- R = {R:.0f} m: direzione (20 s) {g.dir_w20.min():.1f}-{g.dir_w20.max():.1f} gradi, cioe' "
                       f"{g.laterale.min():.1f}-{g.laterale.max():.1f} m di lato; distanza determinata "
                       f"{g.determinata.min():.0f}-{g.determinata.max():.0f}% (al variare di N).")
    if (t.livello == "D").any():
        for n in sorted(t[t.livello == "D"].N.unique()):
            b, dd = t[(t.N == n) & (t.livello == "B")], t[(t.N == n) & (t.livello == "D")]
            if len(b) and len(dd):
                obs.append(f"- N = {n}, sensori B contro D: direzione mediana {b.dir_w20.median():.1f} contro "
                           f"{dd.dir_w20.median():.1f} gradi; distanza determinata {b.determinata.mean():.0f}% contro "
                           f"{dd.determinata.mean():.0f}% (media sulle distanze).")
    txt = ["## 3. Esperimento 2: spoofer esterno", "",
           "**Cosa succede.** A t = 100 s un'antenna a terra (2 m di quota, azimut -37 gradi dal centro dello sciame) "
           "trasmette segnali GNSS falsi che fanno credere a tutti i droni di trovarsi 15 m a nord del centro. Tutti i "
           "ricevitori vengono catturati. Lo sciame se ne accorge confrontando le distanze dichiarate dai GPS con quelle "
           "UWB, passa alla localizzazione solo UWB e localizza l'antenna con le differenze dei PPS (TDoA).",
           "", "**Come lo misuriamo** (`tools/aperture_sweep.py`):",
           "- *consenso*: frazione dei droni che dichiarano lo sciame catturato; *latenza*: dall'attivazione al consenso;",
           "- *direzione*: errore di azimut della stima dello sciame (mediana delle stime dei singoli droni, "
           "'fusione'), usando i primi 20 s dopo la cattura (prima che la deriva dei sensori di bordo ruoti il "
           "riferimento, PAPER_NOTES 4.29); *errore laterale* = R tan(errore di direzione);",
           "- *distanza*: errore mediano tra i droni; *determinata* se l'errore e' minore di R/2; *ideale* = stessa stima "
           "con posizioni e PPS veri (limite dovuto alla sola geometria).", "",
           "**Osservazioni (calcolate dai dati).**", *obs, "",
           "![direzione](fig5_esterno_direzione.png)", "",
           "*Figura 5.* Errore di direzione (sinistra) e laterale (destra) contro la distanza dello spoofer.", "",
           "![distanza](fig6_esterno_distanza.png)", "",
           "*Figura 6.* Frazione di stime di distanza utili (sinistra) ed errore relativo (destra; solo dove la "
           "distanza e' determinata in almeno meta' dei casi; crocette: stima ideale).", ""]
    if "fig7_esterno_sensori.png" in figs:
        txt += ["![sensori](fig7_esterno_sensori.png)", "",
                "*Figura 7.* Sensori di bordo di livello B (buoni, linee continue) contro D (scarsi, tratteggiate).", ""]
    txt += ["*Tabella 2.* Per configurazione (mediane tra i seed; consenso e distanza determinata come media).", "",
            md_table(t, ["N", "R", "livello", "seed", "consenso", "lat_cattura", "dir_tutto", "dir_w20", "dir_w20_q3",
                         "laterale", "dist_rel", "determinata", "ideale_dist_rel", "rotazione", "forma", "apertura"],
                     ["N", "R m", "sensori", "seed", "consenso %", "latenza s", "dir. tutto", "dir. 20 s", "dir. p75",
                      "laterale m", "dist. err %", "determinata %", "ideale %", "rotazione", "forma m", "apertura m"],
                     [None, 0, None, None, 0, 2, 1, 1, 1, 1, 0, 0, 0, 1, 2, 1]), ""]
    return txt, t


# ------------------------------------------------------------------------------------------------------------------
# Esperimento 3: misto
# ------------------------------------------------------------------------------------------------------------------
def report_misto(d, t_ext, rep):
    d, tm = ext_table(d.assign(level="B"))
    rows = []
    for (n, k, R), g in d.groupby(["N", "k", "R"]):
        base = t_ext[(t_ext.N == n) & (t_ext.R == R) & (t_ext.livello == "B")] if t_ext is not None else pd.DataFrame()
        b = base.iloc[0] if len(base) else None
        rows.append(dict(N=n, k=k, R=R, seed=len(g), rilevati=g.attaccanti_rilevati_pct.mean(),
                         latenza=g.latenza_rilevamento_s.median(),
                         consenso=g.consenso_pct.mean(), lat_cattura=g["est.consenso_cattura_latenza_mediana_s"].median(),
                         dir_w20=g["loc.reale_azimut_fusione_W20_gradi"].median(),
                         dir_w20_solo=b.dir_w20 if b is not None else np.nan,
                         determinata=g["loc.reale_distanza_determinata_pct"].mean(),
                         determinata_solo=b.determinata if b is not None else np.nan))
    t = pd.DataFrame(rows).sort_values(["N", "k", "R"])
    t.to_csv(os.path.join(rep, "tab3_misto.csv"), index=False)
    txt = ["## 4. Esperimento 3: prima attaccanti interni, poi spoofer esterno", "",
           "**Cosa succede.** A t = 70 s k = 1 o 2 droni iniziano a mentire sul GPS (direzioni indipendenti); a "
           "t = 140 s si accende lo spoofer esterno e cattura tutti i ricevitori. Domande: l'attaccante interno viene "
           "rilevato? La sua presenza disturba il riconoscimento della cattura e la localizzazione dello spoofer? "
           "Il confronto e' con l'esperimento 2 alla stessa N e R (colonne 'solo spoofer').", "",
           "*Tabella 3.* Attaccanti interni rilevati e latenza; consenso sulla cattura e latenza; direzione (20 s) e "
           "distanza determinata, con e senza attaccanti interni.", "",
           md_table(t, ["N", "k", "R", "seed", "rilevati", "latenza", "consenso", "lat_cattura", "dir_w20", "dir_w20_solo",
                        "determinata", "determinata_solo"],
                    ["N", "k", "R m", "seed", "interni rilevati %", "latenza s", "consenso %", "latenza cattura s",
                     "dir. 20 s", "dir. solo spoofer", "determinata %", "det. solo spoofer %"],
                    [None, None, 0, None, 0, 2, 0, 2, 1, 1, 0, 0]), ""]
    for _, r in t.iterrows():
        if not math.isnan(r.dir_w20_solo):
            txt.append(f"- N = {r.N:.0f}, k = {r.k:.0f}, R = {r.R:.0f} m: interni rilevati {r.rilevati:.0f}%, "
                       f"cattura riconosciuta dal {r.consenso:.0f}% dei droni; direzione {r.dir_w20:.1f} gradi contro "
                       f"{r.dir_w20_solo:.1f} senza attaccanti interni.")
    txt.append("")
    return txt


# ------------------------------------------------------------------------------------------------------------------
# Scalabilita'
# ------------------------------------------------------------------------------------------------------------------
def report_scala(frames, rep, plt):
    d = pd.concat([f for f in frames if f is not None], ignore_index=True)
    g = d.groupby("N").agg(pacchetto=("pacchetto_max_byte", "max"), frame=("frame_ms", "median"),
                           ricezione=("uwb_ricezione_pct", "median"), raggio=("raggio_guscio_m", "median"),
                           tempo=("tempo_run_s", "median")).reset_index()
    g.to_csv(os.path.join(rep, "tab4_scala.csv"), index=False)
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.3))
    axs[0].plot(g.N, g.pacchetto, "o-")
    axs[0].axhline(127, color="r", ls="--", lw=0.8)
    axs[0].text(g.N.min(), 127, " 127 B: trama IEEE 802.15.4 standard", va="bottom", fontsize=7)
    axs[0].axhline(1023, color="k", ls="--", lw=0.8)
    axs[0].text(g.N.min(), 1023, " 1023 B: trama lunga (da verificare sul chip)", va="top", fontsize=7)
    axs[0].set_ylabel("pacchetto piu' grande [byte]")
    axs[1].plot(g.N, g.frame, "o-")
    axs[1].set_ylabel("frame TDMA [ms] (= aggiornamenti per drone ogni ...)")
    axs[2].plot(g.N, g.ricezione, "o-")
    axs[2].set_ylabel("pacchetti UWB ricevuti [%]")
    axs[2].set_ylim(0, 105)
    for ax in axs:
        ax.set_xlabel("droni N")
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "fig8_scala.png"))
    plt.close(fig)
    txt = ["## 5. Scalabilita': fino a dove regge", "",
           "Con un TDMA statico (uno slot da 5 ms per drone) non ci sono collisioni per costruzione: crescendo N "
           "aumentano invece il frame (ogni drone parla una volta ogni N x 5 ms), la dimensione del pacchetto (8 byte "
           "per vicino per il DS-TWR) e il raggio del guscio (quindi le perdite UWB). I limiti fisici da confrontare: "
           "una trama IEEE 802.15.4 standard porta al massimo 127 byte; i chip UWB DW1000/DW3000 ammettono trame piu' "
           "lunghe (fino a 1023 byte in modalita' non standard: da verificare sulla scheda tecnica del chip usato). "
           "Oltre la trama massima il pacchetto andrebbe diviso (piu' slot per drone, frame piu' lungo).", "",
           "![scala](fig8_scala.png)", "",
           "*Figura 8.* Pacchetto piu' grande, durata del frame e pacchetti UWB ricevuti contro N.", "",
           md_table(g, ["N", "pacchetto", "frame", "ricezione", "raggio", "tempo"],
                    ["N", "pacchetto max byte", "frame ms", "ricezione UWB %", "raggio guscio m", "tempo di calcolo s"],
                    [None, 0, 0, 1, 1, 0]), ""]
    return txt


HEADER = """# Campagna per il paper: rapporto automatico

Generato da `tools/paper_report.py` dai risultati di `tools/paper_campaign.py`. Documento di lavoro: i numeri e le
figure si possono usare, il testo va riscritto per il paper.

## 1. Scenario comune a tutte le simulazioni (confrontabilita')

| elemento | scelta | perche' |
|---|---|---|
| geometria | guscio sferico (`--formation=shell`): drone 0 fermo al centro, gli altri su una sfera; 12 m dal vicino piu' prossimo per ogni N (raggio 12 m fino a 10 droni, 16 m con 20, 25 m con 50); centro a 50 m di quota | spaziatura uguale per ogni N; >= 10-15 m serve a riconoscere la cattura (PAPER_NOTES 4.30-4.31); apertura 3D per stimare anche la quota dello spoofer |
| moto | rotazione rigida attorno alla verticale, 1 m/s all'equatore | stessa velocita' lineare per ogni N, come la formazione storica |
| durata, tempi | 240 s; attacco a 100 s (esperimenti 1 e 2); 70 s interni e 140 s esterno (esperimento 3) | 100 s di dati prima dell'attacco per i falsi allarmi, 100-140 s dopo |
| seed | gli stessi per tutte le configurazioni | confronti appaiati |
| radio | `--radio=uwb`: pacchetto ricevuto con probabilita' dal bilancio di collegamento (50% a 85 m, sensibilita' -100 dBm), TDMA statico con slot di 5 ms | PAPER_NOTES 4.20 |

### Sensori simulati

| sensore | modello | parametri |
|---|---|---|
| **UWB** (ranging tra droni, tipo DW3000) | DS-TWR dai timestamp dei pacchetti, orologi a 40 bit (tick 15,65 ps), deriva +-20 ppm; errore realistico (`--uwbError=real`): stato LOS/NLOS correlato nello spazio (3GPP TR 38.901), rumore che cresce con la distanza, riconoscimento NLOS imperfetto | rumore 0,10 m + 0,25 m al limite di portata; NLOS: P_LOS = exp(-d/500), correlazione 10 m, bias U(0,3; 2,5) m; falsi NLOS 5%, NLOS non rilevati 10% (PAPER_NOTES 4.26) |
| **GNSS** (ricevitore di ogni drone) | pseudodistanze con errore comune (atmosfera), multipath e rumore di codice; minimi quadrati per posizione e orologio; impulso PPS | errore comune 1,5 m (600 s), multipath 0,4 m (10 s), codice 0,2 m; PPS con dente di sega +-10 ns e rumore 1 ns (PAPER_NOTES 4.6, 4.15) |
| **velocita' di bordo** (IMU + flusso ottico + bussola), usata solo sotto cattura | velocita' nord/est/su con rumore bianco 0,2 m/s, bias lento (30 s), errore di scala, errore di bussola (120 s) | livello **B** (buoni: terreno ricco di dettagli, bussola compensata): bias 0,1 m/s, scala 1%, bussola 1 grado. Livello **D** (scarsi): 0,3 m/s, 3%, 3 gradi (PAPER_NOTES 4.31-4.32) |
| **spoofer** | antenna singola a terra (2 m), trasmette una costellazione falsa: tutti i ricevitori calcolano la stessa posizione (15 m a nord del centro) e il PPS arriva in ritardo della distanza dall'antenna | distanza R = 25-1000 m, azimut -37 gradi |
| **attaccante interno** | drone dello sciame che dichiara un GPS spostato di 15 m (rampa 10 s) | coordinati (tutti a nord) o indipendenti (direzioni diverse) |
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/paper")
    a = ap.parse_args()
    rep = os.path.join(a.out, "report")
    os.makedirs(rep, exist_ok=True)
    plt = plt_setup()
    di, de, dm = load(a.out, "interni"), load(a.out, "esterno"), load(a.out, "misto")
    counts = []
    for name, d in (("interni", di), ("esterno", de), ("misto", dm)):
        p = os.path.join(a.out, name, "risultati.csv")
        tot = len(pd.read_csv(p)) if os.path.exists(p) else 0
        counts.append(f"{name}: {0 if d is None else len(d)} run riuscite su {tot}")
    lines = [HEADER, "Run disponibili: " + "; ".join(counts) + ".", ""]
    t_ext = None
    if di is not None:
        lines += report_interni(di, rep, plt)
    if de is not None:
        txt, t_ext = report_esterno(de, rep, plt)
        lines += txt
    if dm is not None:
        lines += report_misto(dm, t_ext, rep)
    if any(x is not None for x in (di, de, dm)):
        lines += report_scala([di, de, dm], rep, plt)
    lines += ["## 6. Limiti da dichiarare", "",
              "- Solo simulazione; i parametri dei sensori sono ipotesi documentate (PAPER_NOTES), da calibrare con misure reali.",
              "- Spoofer fermo, a singola antenna; attaccanti interni che mentono solo sulla posizione, con voti onesti.",
              "- Formazione con moto rigido: l'orientamento sotto cattura e' ancorato solo dalle velocita' di bordo "
              "(PAPER_NOTES 4.25, 4.28).",
              "- TDMA statico (uno slot per drone): niente collisioni, ma frame che cresce con N.", ""]
    with open(os.path.join(rep, "report.md"), "w") as fh:
        fh.write("\n".join(lines))
    print(f">>> rapporto: {os.path.join(rep, 'report.md')} ({len(os.listdir(rep))} file)")


if __name__ == "__main__":
    main()
