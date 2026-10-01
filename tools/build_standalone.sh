#!/usr/bin/env bash
# Compila il simulatore FUORI dall'albero di ns-3, contro un'installazione di ns-3
# che fornisca header (include/ns3/*.h) e librerie (libns3.XX-*.so).
#
# Uso:
#   NS3_PREFIX=/percorso/ns3 NS3_VER=3.44 tools/build_standalone.sh [output]
#
# Con ns-3 installato da pip (pip install ns3):
#   NS3_PREFIX=$(python3 -c "import importlib.util;print(importlib.util.find_spec('ns3').submodule_search_locations[0])")
#
# Il flusso normale (cartella scratch/ di ns-3 + CMakeLists.txt) resta valido:
# questo script serve solo per compilare in fretta e per i test automatici.
set -euo pipefail

: "${NS3_PREFIX:?imposta NS3_PREFIX (cartella con include/ e lib64/ o lib/)}"
NS3_VER="${NS3_VER:-3.44}"
OUTPUT="${1:-sim}"
EIGEN_INC="${EIGEN_INC:-/usr/include/eigen3}"

LIBDIR="$NS3_PREFIX/lib64"; [[ -d "$LIBDIR" ]] || LIBDIR="$NS3_PREFIX/lib"

MODULES="netanim wifi spectrum antenna propagation energy applications internet \
bridge traffic-control mobility network stats core"
LIBS=""; for m in $MODULES; do LIBS="$LIBS -lns$NS3_VER-$m"; done

cd "$(dirname "$0")/.."
g++ -std=c++20 -O2 -Wall -Wno-unused-parameter \
    -I"$NS3_PREFIX/include" -I"$EIGEN_INC" \
    main.cpp UwbSecurityApp.cpp UwbHeader.cpp Trajectories.cpp EKF.cpp UWBChannel.cpp \
    -L"$LIBDIR" -Wl,-rpath,"$LIBDIR" -Wl,--allow-shlib-undefined $LIBS \
    -o "$OUTPUT"
echo "Compilato: $(pwd)/$OUTPUT"
