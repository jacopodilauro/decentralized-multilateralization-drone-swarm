# Conta le run della campagna del paper: previste (profilo completo) contro riuscite, per esperimento e numero di droni.
# Uso (da ~/pleion/src):
#   python3 conta_run.py [cartella]            conteggio + elenco delle run mancanti con lo stato della cartella
#   python3 conta_run.py [cartella] --pulisci  cancella le cartelle delle run finite con errore (rc != 0), cosi'
#                                              la campagna le rifa' (le run riuscite non vengono mai toccate)
import json, os, shutil, sys
import pandas as pd
sys.path.insert(0, "tools")
import paper_campaign as p

args = [a for a in sys.argv[1:] if not a.startswith("--")]
base = args[0] if args else "runs/paper"
pulisci = "--pulisci" in sys.argv
key = ["exp", "N", "k", "mode", "R", "level", "seed"]

def norm(df):
    df = df.copy()
    for c in ("N", "k", "seed", "R"):
        df[c] = pd.to_numeric(df[c], errors="coerce").map(lambda v: "" if pd.isna(v) else str(int(v)))
    for c in ("exp", "mode", "level"):
        df[c] = df[c].fillna("").astype(str)
    return df[key]

cfg = p.configs("completo", ["interni", "esterno", "misto"])
piano = norm(pd.DataFrame(cfg))
fatte = []
for e in ("interni", "esterno", "misto"):
    f = os.path.join(base, e, "risultati.csv")
    if os.path.exists(f):
        d = pd.read_csv(f)
        fatte.append(d[d.rc == 0] if "rc" in d else d)
fatte = norm(pd.concat(fatte)).drop_duplicates() if fatte else pd.DataFrame(columns=key)

piano["fatta"] = piano.merge(fatte.assign(_ok=1), on=key, how="left")["_ok"].fillna(0).astype(int).values
tab = piano.groupby(["exp", "N"]).fatta.agg(previste="size", fatte="sum")
tab["mancanti"] = tab.previste - tab.fatte
tab = tab.reset_index()
tab["N"] = tab.N.astype(int)
tab = tab.sort_values(["exp", "N"]).set_index(["exp", "N"])
print(tab.to_string())

mancanti = [c for c, ok in zip(cfg, piano.fatta) if not ok]
if mancanti:
    print(f"\nRUN MANCANTI ({len(mancanti)}):")
    da_pulire = []
    for c in mancanti:
        d = os.path.join(base, c["exp"], p.label(c), f"seed{c['seed']}")
        rowf = os.path.join(d, "row.json")
        if not os.path.isdir(d):
            stato = "mai iniziata"
        elif not os.path.exists(rowf):
            stato = "interrotta (nessun row.json): la campagna la rifa' da sola"
        else:
            r = json.load(open(rowf))
            stato = f"salvata CON ERRORE (rc={r.get('rc')}{', ' + str(r['errore'])[:60] if r.get('errore') else ''}): la campagna la considera fatta"
            da_pulire.append(d)
        print(f"  {d}: {stato}")
    if da_pulire and pulisci:
        for d in da_pulire:
            shutil.rmtree(d)
        print(f"\nCancellate {len(da_pulire)} cartelle con errore: rilancia la campagna per rifarle.")
    elif da_pulire:
        print(f"\n{len(da_pulire)} run salvate con errore: per farle rifare lancia  python3 conta_run.py {base} --pulisci  e poi la campagna.")
print(f"\nTOTALE: {int(tab.fatte.sum())} riuscite su {int(tab.previste.sum())} previste, mancano {int(tab.mancanti.sum())}")
