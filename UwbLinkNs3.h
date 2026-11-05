#pragma once
// Attenuazione media del collegamento UWB con il modello di ns-3 (LogDistancePropagationLossModel):
//   Pr = EIRP - L0 - 10 n log10(d / 1 m) - perdita d'antenna, con L0 = spazio libero a 1 m (Friis) e n di UwbLink.h.
// E' la stessa formula di uwblink::MeanRxDbm (che resta per gli usi dove si conosce solo una distanza: rumore del
// canale e varianza nel filtro, calcolata dalla distanza MISURATA). ns-3 calcola EIRP - (L0 + 10 n log10 d), UwbLink.h
// (EIRP - L0) - 10 n log10 d: stessa formula, ultima cifra a volte diversa (scarto <= 1,5e-14 dB, verificato in
// tests/test_uwblink_ns3.cc insieme all'ordine esatto delle operazioni di ns-3). L'ombreggiamento e la ricezione restano in UwbLink.h: RandomPropagationLossModel
// ha un solo generatore per tutto il canale (il difetto corretto nella Fase 2.0) e ns-3 non ha un livello fisico UWB HRP.
#include "UwbLink.h"
#include "ns3/mobility-model.h"
#include "ns3/propagation-loss-model.h"

namespace uwblink {

inline ns3::Ptr<ns3::LogDistancePropagationLossModel> MakeNs3Loss(const Params& p) {
    auto m = ns3::CreateObject<ns3::LogDistancePropagationLossModel>();
    m->SetPathLossExponent(p.n);
    m->SetReference(1.0, Pl0Db(p));
    return m;
}

inline double MeanRxDbmNs3(const ns3::Ptr<ns3::PropagationLossModel>& loss, const Params& p,
                           ns3::Ptr<ns3::MobilityModel> tx, ns3::Ptr<ns3::MobilityModel> rx) {
    return loss->CalcRxPower(TxEirpDbm(p), tx, rx) - p.antennaLossDb;
}

}  // namespace uwblink
