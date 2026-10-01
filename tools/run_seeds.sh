#!/usr/bin/env bash
# Esegue la simulazione con piu' seed, calcola le metriche e le aggrega.
#
# Uso:
#   SIM=/percorso/eseguibile tools/run_seeds.sh <etichetta> [argomenti extra per il simulatore]
#
# Esempi:
#   SIM=./sim tools/run_seeds.sh baseline
#   SIM=./sim SEEDS="1 2 3 4 5" tools/run_seeds.sh baseline --setSimTime=240
#
# Variabili d'ambiente:
#   SIM          eseguibile del simulatore (obbligatorio)
#   SEEDS        elenco dei runId (default: "1 2 3")
#   ATTACK_TIME  istante dell'attacco per le metriche (default: 200)
#   TARGETS      ID attaccati, separati da virgola (default: 0)
#   N_BASE       numero di droni base (default: 8)
#
# Risultati in runs/<etichetta>/seed<N>/ e runs/<etichetta>/summary.json
set -euo pipefail

if [[ $# -lt 1 || -z "${SIM:-}" ]]; then
    sed -n '2,20p' "$0"; exit 2
fi

LABEL="$1"; shift
SEEDS="${SEEDS:-1 2 3}"
ATTACK_TIME="${ATTACK_TIME:-200}"
TARGETS="${TARGETS:-0}"
N_BASE="${N_BASE:-8}"

TOOLS="$(cd "$(dirname "$0")" && pwd)"
SIM_ABS="$(cd "$(dirname "$SIM")" && pwd)/$(basename "$SIM")"
OUT="runs/$LABEL"
mkdir -p "$OUT"

JSONS=()
for s in $SEEDS; do
    D="$OUT/seed$s"
    mkdir -p "$D"
    echo ">>> [$LABEL] seed $s ..."
    START=$(date +%s)
    ( cd "$D" && "$SIM_ABS" --runId="$s" --netanim=0 "$@" > stdout.txt 2>&1 )
    echo "    completato in $(( $(date +%s) - START )) s"
    python3 "$TOOLS/metrics.py" "$D/tdma_security_log.csv" \
        --attack-time "$ATTACK_TIME" --targets "$TARGETS" --n-base "$N_BASE" \
        --out "$D/metrics.json" > /dev/null
    JSONS+=("$D/metrics.json")
done

python3 "$TOOLS/metrics.py" --aggregate "${JSONS[@]}" --out "$OUT/summary.json" > /dev/null
echo ">>> Metriche aggregate in $OUT/summary.json"
