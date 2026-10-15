#!/usr/bin/env bash
# Strada A: campagna "distanza dello spoofer in funzione della scala della formazione".
# Per ogni combinazione scala S x distanza R x seed: una simulazione con lo spoofer, l'analisi con
# tools/aperture_sweep.py (una riga per run) e la cancellazione dei CSV grandi. Simulazioni in parallelo.
#
# Uso (dalla cartella del simulatore):
#   SIM=/percorso/eseguibile SCALES="1 2 3 4" DISTS="250 500 1000" SEEDS="1 2 3 4 5" JOBS=6 \
#       tools/run_aperture_sweep.sh [etichetta]
# Risultati: runs/<etichetta>/risultati.csv; figura:
#   python3 tools/aperture_sweep.py --plot runs/<etichetta>/risultati.csv --png figura.png
# Default: SCALES="1 2 3 4", DISTS="500", SEEDS="1 2 3", JOBS = numero di core, etichetta "aperture".
# EXTRA: opzioni aggiuntive per il simulatore, es. EXTRA="--radio=uwb" (Fase 3).
set -euo pipefail

if [[ -z "${SIM:-}" ]]; then sed -n '2,12p' "$0"; exit 2; fi
LABEL="${1:-aperture}"
SCALES="${SCALES:-1 2 3 4}"
DISTS="${DISTS:-500}"
SEEDS="${SEEDS:-1 2 3}"
JOBS="${JOBS:-$(nproc)}"
TOOLS="$(cd "$(dirname "$0")" && pwd)"
SIM_ABS="$(cd "$(dirname "$SIM")" && pwd)/$(basename "$SIM")"
OUT="runs/$LABEL"
# L'eseguibile deve conoscere --formationScale (strada A): altrimenti e' una build vecchia (ricompilare ns-3)
HELP="$("$SIM_ABS" --PrintHelp 2>&1 || true)"   # prima in una variabile: con pipefail, grep -q chiuderebbe la pipe
if ! grep -q -- "--formationScale" <<< "$HELP"; then
    echo "ERRORE: $SIM_ABS non conosce --formationScale: e' una build vecchia."
    echo "        Ricompila dopo git am:  cd ~/ns-3-dev && ./ns3 build"
    exit 1
fi
mkdir -p "$OUT"

one() {   # $1 = scala, $2 = distanza, $3 = seed
    local d="$OUT/S$1_R$2/seed$3"
    mkdir -p "$d"
    if [[ -f "$d/aperture.csv" ]]; then echo "    gia' fatto: $d"; return 0; fi
    local rc=0
    ( cd "$d" && "$SIM_ABS" --runId="$3" --netanim=0 --verbose=1 --spoof=1 --spoofTime=100 \
          --spoofDist="$2" --formationScale="$1" $EXTRA > stdout.txt 2>&1 ) || rc=$?
    if [[ $rc -ne 0 ]]; then
        echo "    ERRORE S=$1 R=$2 seed $3: il simulatore e' uscito con codice $rc: $(head -1 "$d/stdout.txt")"
        return 1
    fi
    if ! python3 "$TOOLS/aperture_sweep.py" --one "$d" --out "$d/aperture.csv" > "$d/analisi.txt" 2>&1; then
        echo "    ERRORE S=$1 R=$2 seed $3: analisi fallita: $(tail -1 "$d/analisi.txt") (CSV lasciati in $d)"
        return 1
    fi
    rm -f "$d/tdma_security_log.csv" "$d/gnss_log.csv" "$d/pps_log.csv"
    echo "    fatto: S=$1 R=$2 seed $3"
}
export -f one
EXTRA="${EXTRA:-}"
export OUT SIM_ABS TOOLS EXTRA

echo ">>> [$LABEL] scale: $SCALES | distanze: $DISTS | seed: $SEEDS | in parallelo: $JOBS | opzioni: ${EXTRA:-nessuna}"
for s in $SCALES; do for r in $DISTS; do for n in $SEEDS; do echo "$s $r $n"; done; done; done \
    | xargs -P "$JOBS" -L 1 bash -c 'one $0 $1 $2' || true   # gli errori sono gia' stati scritti, run per run

python3 - "$OUT" "$SCALES" "$DISTS" "$SEEDS" <<'EOF'
import glob, os, sys, pandas as pd
out, scales, dists, seeds = sys.argv[1], sys.argv[2].split(), sys.argv[3].split(), sys.argv[4].split()
attese = [f"{out}/S{s}_R{r}/seed{n}" for s in scales for r in dists for n in seeds]
mancanti = [d for d in attese if not os.path.exists(f"{d}/aperture.csv")]
parts = [pd.read_csv(f) for f in sorted(glob.glob(f"{out}/S*_R*/seed*/aperture.csv"))]
print(f">>> run riuscite: {len(attese) - len(mancanti)} su {len(attese)}")
for d in mancanti:
    print(f"    mancante: {d} (vedi {d}/stdout.txt)")
if not parts:
    print(">>> nessun risultato: niente da unire"); sys.exit(1)
pd.concat(parts, ignore_index=True).to_csv(f"{out}/risultati.csv", index=False)
print(f">>> {len(parts)} run in {out}/risultati.csv (rilanciando lo script si rifanno solo le mancanti)")
sys.exit(1 if mancanti else 0)
EOF
