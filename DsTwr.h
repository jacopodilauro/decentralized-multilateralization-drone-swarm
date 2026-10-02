#ifndef DS_TWR_H
#define DS_TWR_H

// Double-sided two-way ranging (DS-TWR) asimmetrico.
// Tre messaggi tra A e B:  A --m1--> B,  B --m2--> A,  A --m3--> B
//   tA1 = A trasmette m1   tB1 = B riceve m1
//   tA2 = A riceve m2      tB2 = B trasmette m2
//   tA3 = A trasmette m3   tB3 = B riceve m3
// Ogni intervallo e' misurato da UN SOLO orologio, quindi l'offset tra gli orologi sparisce:
//   Ra = tA2 - tA1 (A)   Da = tA3 - tA2 (A)   Rb = tB3 - tB2 (B)   Db = tB2 - tB1 (B)
//   ToF = (Ra*Rb - Da*Db) / (Ra + Rb + Da + Db)
// L'effetto della deriva degli orologi resta dell'ordine di ToF * (differenza di skew): trascurabile.
// Con droni in movimento, al primo ordine il risultato e' la distanza all'istante del messaggio
// centrale (tB2): ToF ~ 1/2 t2 + Da/(2(Da+Db)) t1 + Db/(2(Da+Db)) t3, che con velocita' costante
// vale esattamente il tempo di volo a tB2. Verificato in tests/test_dstwr.cc.

#include "ClockModel.h"

namespace uwbclock {

struct DsTwrStamps {
    uint64_t tA1, tA2, tA3;   // orologio di A
    uint64_t tB1, tB2, tB3;   // orologio di B
};

// Tempo di volo [s]. Ritorna un valore negativo se gli intervalli non sono coerenti.
inline double DsTwrTof(const DsTwrStamps& s)
{
    const long double Ra = TickDiff(s.tA1, s.tA2);
    const long double Da = TickDiff(s.tA2, s.tA3);
    const long double Rb = TickDiff(s.tB2, s.tB3);
    const long double Db = TickDiff(s.tB1, s.tB2);
    if (Ra <= 0 || Da <= 0 || Rb <= 0 || Db <= 0) return -1.0;
    const long double tofTicks = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
    return static_cast<double>(tofTicks) * TICK_S;
}

inline double DsTwrRange(const DsTwrStamps& s) { return DsTwrTof(s) * LIGHT_C; }

} // namespace uwbclock

#endif
