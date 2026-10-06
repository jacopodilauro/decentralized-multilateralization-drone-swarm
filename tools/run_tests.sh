#!/usr/bin/env bash
# Compila ed esegue tutti i test isolati (cartella tests/).
#
# Uso:
#   NS3_PREFIX=/percorso/ns3 NS3_VER=3.44 tools/run_tests.sh
#
# I test che non usano ns-3 (EKF, DS-TWR) girano sempre. Quelli che usano ns-3 (header, canale)
# richiedono NS3_PREFIX (cartella con include/ e lib/ o lib64/) e NS3_VER (es. 3.44 o dev);
# se NS3_PREFIX non e' impostato vengono saltati.
set -uo pipefail
cd "$(dirname "$0")/.."
EIGEN_INC="${EIGEN_INC:-/usr/include/eigen3}"
OUT="$(mktemp -d)"
fail=0

run() {   # nome, comando di compilazione
    local name="$1"; shift
    if ! "$@" -o "$OUT/$name" 2> "$OUT/$name.log"; then
        echo "[$name] ERRORE DI COMPILAZIONE (vedi $OUT/$name.log)"; fail=1; return
    fi
    local last; last="$("$OUT/$name" | tail -1)"
    echo "[$name] $last"
    [[ "$last" == *"TUTTI I TEST SUPERATI"* ]] || fail=1
}

run test_ekf    g++ -std=c++20 -O2 -I"$EIGEN_INC" tests/test_ekf.cc EKF.cpp
run test_dstwr  g++ -std=c++20 -O2 -I"$EIGEN_INC" tests/test_dstwr.cc
run test_gnss   g++ -std=c++20 -O2 -I"$EIGEN_INC" tests/test_gnss.cc

if [[ -n "${NS3_PREFIX:-}" ]]; then
    NS3_VER="${NS3_VER:-3.44}"
    LIBDIR="$NS3_PREFIX/lib64"; [[ -d "$LIBDIR" ]] || LIBDIR="$NS3_PREFIX/lib"
    NS="-I$NS3_PREFIX/include -L$LIBDIR -Wl,-rpath,$LIBDIR -Wl,--allow-shlib-undefined"
    run test_header  g++ -std=c++20 $NS tests/test_header.cc UwbHeader.cpp -lns$NS3_VER-network -lns$NS3_VER-core
    run test_channel g++ -std=c++20 $NS -I"$EIGEN_INC" tests/test_channel.cc UWBChannel.cpp -lns$NS3_VER-core
else
    echo "[test_header, test_channel] saltati: imposta NS3_PREFIX per eseguirli"
fi

rm -rf "$OUT"
[[ $fail -eq 0 ]] && echo "== tutti i test isolati superati" || echo "== ALCUNI TEST SONO FALLITI"
exit $fail
