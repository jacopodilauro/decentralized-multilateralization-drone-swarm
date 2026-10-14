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
mkdir -p "$OUT"

one() {   # $1 = scala, $2 = distanza, $3 = seed
    local d="$OUT/S$1_R$2/seed$3"
    mkdir -p "$d"
    if [[ -f "$d/aperture.csv" ]]; then echo "    gia' fatto: $d"; return 0; fi
    ( cd "$d" && "$SIM_ABS" --runId="$3" --netanim=0 --verbose=1 --spoof=1 --spoofTime=100 \
          --spoofDist="$2" --formationScale="$1" > stdout.txt 2>&1 )
    python3 "$TOOLS/aperture_sweep.py" --one "$d" --out "$d/aperture.csv" > /dev/null
    rm -f "$d/tdma_security_log.csv" "$d/gnss_log.csv" "$d/pps_log.csv"
    echo "    fatto: S=$1 R=$2 seed $3"
}
export -f one
export OUT SIM_ABS TOOLS

echo ">>> [$LABEL] scale: $SCALES | distanze: $DISTS | seed: $SEEDS | in parallelo: $JOBS"
for s in $SCALES; do for r in $DISTS; do for n in $SEEDS; do echo "$s $r $n"; done; done; done \
    | xargs -P "$JOBS" -L 1 bash -c 'one $0 $1 $2'

python3 - "$OUT" <<'EOF'
import glob, sys, pandas as pd
out = sys.argv[1]
parts = [pd.read_csv(f) for f in sorted(glob.glob(f"{out}/S*_R*/seed*/aperture.csv"))]
pd.concat(parts, ignore_index=True).to_csv(f"{out}/risultati.csv", index=False)
print(f">>> {len(parts)} run in {out}/risultati.csv")
EOF
