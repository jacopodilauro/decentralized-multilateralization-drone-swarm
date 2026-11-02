#!/usr/bin/env python3
"""Grafici in LaTeX (pgfplots) dai risultati della campagna: stesso font del documento, modificabili in Overleaf.

Uso:  python3 paper_pgfplots.py --report runs/paper/report --out grafici [--lingua en]
Legge tab1_interni.csv e tab2_esterno.csv (scritti da tools/paper_report.py) e scrive un file .tex per grafico, con i dati
gia' dentro (nessun file esterno), piu' anteprima.tex che li mostra tutti.

Nel documento (preambolo):
    \\usepackage{pgfplots}
    \\pgfplotsset{compat=1.18}
    \\usepgfplotslibrary{groupplots}
e dove serve il grafico:
    \\begin{figure}[h]\\centering \\input{grafici/fig_distanza.tex} \\caption{...}\\end{figure}

Criterio "regge" (griglia): attaccanti rilevati >= 90% E falsi allarmi sugli onesti <= 5% (mediana tra i seed).
"""
import argparse
import os

import pandas as pd

SIZES = [5, 6, 8, 12, 20, 30, 50]
OKABE = {5: "E69F00", 6: "56B4E9", 8: "009E73", 12: "CC79A7", 20: "0072B2", 30: "D55E00", 50: "000000"}
MARK = {5: "*", 6: "square*", 8: "triangle*", 12: "diamond*", 20: "pentagon*", 30: "star", 50: "x"}
R_TICKS = "25,50,100,200,500,1000"

T = {
    "it": dict(N="droni $N$", k="attaccanti interni $k$", coord="attaccanti coordinati", indip="attaccanti indipendenti",
               holds="regge", fails="cede", incomplete="seed incompleti", line5="$k = N-5$", line3="$k = N/3$ (voto bizantino)",
               lat="latenza di rilevamento [s]", one="un attaccante", band="tutti i $k$ che reggono",
               R="distanza dello spoofer $R$ [m]", lateral="errore laterale [m]", resolved="distanza riuscita [\\%]",
               relerr="errore di distanza / $R$ [\\%]", ideal="limite geometrico", bearing="errore di direzione [$^\\circ$]",
               good="sensori buoni", poor="sensori scarsi"),
    "en": dict(N="swarm size $N$", k="insiders $k$", coord="coordinated insiders", indip="independent insiders",
               holds="holds", fails="fails", incomplete="incomplete seeds", line5="$k = N-5$", line3="$k = N/3$ (Byzantine voting)",
               lat="detection latency [s]", one="one insider", band="all $k$ where the system holds",
               R="spoofer distance $R$ [m]", lateral="cross-range error [m]", resolved="range resolved [\\%]",
               relerr="range error / $R$ [\\%]", ideal="geometric bound", bearing="bearing error [deg]",
               good="good sensors", poor="poor sensors"),
}

COMMON = r"""\pgfplotsset{every axis/.append style={tick label style={font=\small}, label style={font=\small},
  legend style={font=\footnotesize, draw=black!30, fill=white, fill opacity=0.9, text opacity=1},
  grid=major, grid style={black!12}, /pgf/number format/1000 sep={}}}
"""


def colors():
    return "".join(f"\\definecolor{{N{n}}}{{HTML}}{{{c}}}\n" for n, c in OKABE.items()) + \
        "\\definecolor{regge}{HTML}{009E73}\n\\definecolor{cede}{HTML}{D55E00}\n"


def coords(xs, ys):
    return " ".join(f"({x:g},{y:.4g})" for x, y in zip(xs, ys) if pd.notna(x) and pd.notna(y))


def write(out, name, body):
    with open(os.path.join(out, name), "w") as fh:
        fh.write(f"% Generato da paper_pgfplots.py: non modificare a mano i dati, rigenera dal rapporto della campagna\n")
        fh.write(colors() + body)
    return name


# ------------------------------------------------------------------------------------------- attaccanti interni
def fig_griglia(t1, out, L):
    t = t1.copy()
    t["regge"] = (t.rilevati >= 90) & (t.fa_med <= 5)
    t["completa"] = t.seed >= t.groupby("N").seed.transform("max")
    Ns = [n for n in SIZES if n in set(t.N)]
    xi = {n: i for i, n in enumerate(Ns)}
    ymax = max(t.k.max() + 1.5, 5)
    panels = []
    for mode, title in (("coordinati", L["coord"]), ("indipendenti", L["indip"])):
        tm = t[t["mode"] == mode]
        plots = []
        for cls, col, fill in (("regge", "regge", True), ("cede", "cede", True)):
            for complete in (True, False):
                g = tm[(tm.regge == (cls == "regge")) & (tm.completa == complete)]
                if len(g) == 0:
                    continue
                style = f"only marks, mark=square*, mark size=2.6pt, color={col}" + ("" if complete else ", mark options={fill=white}")
                plots.append(f"    \\addplot[{style}] coordinates {{{coords([xi[n] for n in g.N], g.k)}}};")
        plots.append(f"    \\addplot[black, dashed, thick] coordinates {{{coords(range(len(Ns)), [n - 5 for n in Ns])}}};")
        plots.append(f"    \\addplot[black!60, dotted, thick] coordinates {{{coords(range(len(Ns)), [n / 3 for n in Ns])}}};")
        panels.append((title, plots))
    ticks = ",".join(str(i) for i in range(len(Ns)))
    labels = ",".join(str(n) for n in Ns)
    body = COMMON + f"""\\begin{{tikzpicture}}
\\begin{{groupplot}}[group style={{group size=2 by 1, horizontal sep=1.2cm, y descriptions at=edge left}},
  width=0.47\\linewidth, height=6cm, xmin=-0.5, xmax={len(Ns) - 0.5}, ymin=0, ymax={ymax:g},
  xtick={{{ticks}}}, xticklabels={{{labels}}}, xlabel={{{L['N']}}}, ylabel={{{L['k']}}}]
"""
    for i, (title, plots) in enumerate(panels):
        body += f"  \\nextgroupplot[title={{{title}}}, title style={{font=\\small}}"
        if i == 1:
            body += f""", legend style={{at={{(-0.12,-0.28)}}, anchor=north, legend columns=4}}, legend entries={{{L['holds']},{L['fails']},{L['line5']},{L['line3']}}}"""
        body += "]\n"
        if i == 1:   # voci di legenda fisse, in ordine: regge, cede, k = N-5, k = N/3
            body += ("    \\addlegendimage{only marks, mark=square*, mark size=2.6pt, color=regge}\n"
                     "    \\addlegendimage{only marks, mark=square*, mark size=2.6pt, color=cede}\n"
                     "    \\addlegendimage{black, dashed, thick}\n    \\addlegendimage{black!60, dotted, thick}\n")
            plots = [p.replace("\\addplot[", "\\addplot[forget plot, ") for p in plots]
        body += "\n".join(plots) + "\n"
    body += "\\end{groupplot}\n\\end{tikzpicture}\n"
    return write(out, "fig_griglia_interni.tex", body)


def fig_latenza(t1, out, L):
    one = t1[(t1.k == 1) & (t1["mode"] == "indipendenti")].sort_values("N")
    h = t1[(t1.rilevati >= 90) & (t1.fa_med <= 5)].groupby("N").latenza.agg(["min", "max"]).dropna()
    band = " -- ".join(f"(axis cs:{n:g},{v:.4g})" for n, v in zip(h.index, h["max"]))
    band += " -- " + " -- ".join(f"(axis cs:{n:g},{v:.4g})" for n, v in zip(h.index[::-1], h["min"][::-1])) + " -- cycle"
    body = COMMON + f"""\\begin{{tikzpicture}}
\\begin{{axis}}[width=0.62\\linewidth, height=5cm, xlabel={{{L['N']}}}, ylabel={{{L['lat']}}}, ymin=0,
  xtick={{{",".join(str(int(n)) for n in one.N if int(n) != 6)}}}, legend pos=south east]
  \\fill[N20, opacity=0.18] {band};
  \\addlegendimage{{area legend, fill=N20, fill opacity=0.18, draw=none}}
  \\addlegendentry{{{L['band']}}}
  \\addplot[N20, thick, mark=*] coordinates {{{coords(one.N, one.latenza)}}};
  \\addlegendentry{{{L['one']}}}
\\end{{axis}}
\\end{{tikzpicture}}
"""
    return write(out, "fig_latenza_interni.tex", body)


# ------------------------------------------------------------------------------------------- spoofer esterno
def series_by_N(t, ycol, extra=""):
    lines = []
    for n, g in t.groupby("N"):
        g = g.sort_values("R")
        lines.append(f"    \\addplot[N{n}, thick, mark={MARK.get(n, '*')}{extra}] coordinates {{{coords(g.R, g[ycol])}}};")
        lines.append(f"    \\addlegendentry{{$N = {n}$}}")
    return "\n".join(lines)


def fig_direzione(t2, out, L):
    t = t2[t2.livello == "B"]
    body = COMMON + f"""\\begin{{tikzpicture}}
\\begin{{axis}}[width=0.62\\linewidth, height=5.4cm, xmode=log, ymode=log, log ticks with fixed point,
  xtick={{{R_TICKS}}}, xlabel={{{L['R']}}}, ylabel={{{L['lateral']}}}, legend pos=north west, legend columns=2]
{series_by_N(t, 'laterale')}
\\end{{axis}}
\\end{{tikzpicture}}
"""
    return write(out, "fig_direzione.tex", body)


def fig_distanza(t2, out, L):
    t = t2[t2.livello == "B"]
    left = series_by_N(t, "determinata")
    right = []
    for n, g in t.groupby("N"):
        g = g.sort_values("R")
        ok = g[g.determinata >= 50]
        if len(ok):
            right.append(f"    \\addplot[N{n}, thick, mark={MARK.get(n, '*')}, forget plot] coordinates {{{coords(ok.R, ok.dist_rel)}}};")
        right.append(f"    \\addplot[N{n}, densely dotted, thick, forget plot] coordinates {{{coords(g.R, g.ideale_dist_rel)}}};")
    body = COMMON + f"""\\begin{{tikzpicture}}
\\begin{{groupplot}}[group style={{group size=2 by 1, horizontal sep=1.6cm}}, width=0.47\\linewidth, height=5.4cm,
  xmode=log, log ticks with fixed point, xtick={{{R_TICKS}}}, xlabel={{{L['R']}}}]
  \\nextgroupplot[ylabel={{{L['resolved']}}}, ymin=-3, ymax=103, legend style={{at={{(0.5,-0.3)}}, anchor=north}}, legend columns=3]
{left}
  \\nextgroupplot[ylabel={{{L['relerr']}}}, ymode=log, log ticks with fixed point,
    legend style={{at={{(0.03,0.97)}}, anchor=north west}}]
    \\addlegendimage{{black, thick}}\\addlegendentry{{stima}}
    \\addlegendimage{{black, densely dotted, thick}}\\addlegendentry{{{L['ideal']}}}
{chr(10).join(right)}
\\end{{groupplot}}
\\end{{tikzpicture}}
"""
    if L is T["en"]:
        body = body.replace("{stima}", "{estimated}")
    return write(out, "fig_distanza.tex", body)


def fig_sensori(t2, out, L):
    nd = sorted(t2[t2.livello == "D"].N.unique())
    if not nd:
        return None
    pans = {"dir_w20": [], "determinata": []}
    for n in nd:
        for lev, ls, lab in (("B", "solid", L["good"]), ("D", "dashed", L["poor"])):
            g = t2[(t2.N == n) & (t2.livello == lev)].sort_values("R")
            for col in pans:
                pans[col].append(f"    \\addplot[N{n}, thick, {ls}, mark={MARK.get(n, '*')}] coordinates {{{coords(g.R, g[col])}}};")
                if col == "determinata":
                    pans[col].append(f"    \\addlegendentry{{$N = {n}$, {lab}}}")
    body = COMMON + f"""\\begin{{tikzpicture}}
\\begin{{groupplot}}[group style={{group size=2 by 1, horizontal sep=1.6cm}}, width=0.47\\linewidth, height=5.4cm,
  xmode=log, log ticks with fixed point, xtick={{{R_TICKS}}}, xlabel={{{L['R']}}}]
  \\nextgroupplot[ylabel={{{L['bearing']}}}, ymin=0]
{chr(10).join(pans['dir_w20'])}
  \\nextgroupplot[ylabel={{{L['resolved']}}}, ymin=-3, ymax=103, legend style={{at={{(0.5,-0.3)}}, anchor=north}}, legend columns=2]
{chr(10).join(pans['determinata'])}
\\end{{groupplot}}
\\end{{tikzpicture}}
"""
    return write(out, "fig_sensori.tex", body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", default="runs/paper/report")
    ap.add_argument("--out", default="grafici")
    ap.add_argument("--lingua", default="it", choices=["it", "en"])
    a = ap.parse_args()
    L = T[a.lingua]
    os.makedirs(a.out, exist_ok=True)
    rd = lambda n: pd.read_csv(os.path.join(a.report, n)) if os.path.exists(os.path.join(a.report, n)) else None  # noqa: E731
    t1, t2 = rd("tab1_interni.csv"), rd("tab2_esterno.csv")
    made = []
    if t1 is not None:
        made += [fig_griglia(t1, a.out, L), fig_latenza(t1, a.out, L)]
    if t2 is not None:
        made += [fig_direzione(t2, a.out, L), fig_distanza(t2, a.out, L), fig_sensori(t2, a.out, L)]
    made = [m for m in made if m]
    pre = r"""\documentclass[11pt,a4paper]{article}
\usepackage[a4paper,margin=2.5cm]{geometry}
\usepackage{pgfplots}
\pgfplotsset{compat=1.18}
\usepgfplotslibrary{groupplots}
\begin{document}
"""
    doc = pre + "".join(f"\\section*{{\\texttt{{\\detokenize{{{m}}}}}}}\n\\begin{{center}}\n\\input{{{m}}}\n\\end{{center}}\n\n"
                        for m in made) + "\\end{document}\n"
    with open(os.path.join(a.out, "anteprima.tex"), "w") as fh:
        fh.write(doc)
    print(">>> scritti in " + a.out + ": " + ", ".join(made) + ", anteprima.tex")


if __name__ == "__main__":
    main()
