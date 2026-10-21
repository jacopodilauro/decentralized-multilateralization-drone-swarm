#pragma once
// Errore di misura UWB realistico (Fase 5), attivo con --uwbError=real (default legacy: modello storico).
//
// 1) Stato LOS/NLOS correlato (procedura di consistenza spaziale di 3GPP TR 38.901, par. 7.6.3.1 e 7.6.3.3):
//    per ogni collegamento NON orientato (stesso stato nei due versi) una variabile gaussiana x(s) funzione dello
//    spostamento cumulato s dei due estremi (nodi indipendenti ogni L = corrDist / 0,905 m, interpolati e
//    rinormalizzati: correlazione 1/e a corrDist); il collegamento e' LOS se Phi(x) < P_LOS(d). Il processo e'
//    continuo, quindi la durata degli episodi non dipende dalla frequenza dei pacchetti. La frazione NLOS per distanza resta quella storica, P_LOS = exp(-d / losDist);
//    cambia solo la memoria. corrDist = 0 -> stati indipendenti a ogni messaggio.
//    Il bias NLOS (eccesso di ritardo del primo cammino) resta lo stesso per tutto l'episodio.
// 2) Rumore che cresce con la distanza: varianza per messaggio sigmaFloor^2 + sigmaEdge^2 10^(-M/10), con M
//    margine [dB] tra potenza media ricevuta (bilancio della Fase 3, UwbLink.h) e sensibilita'. Il secondo
//    termine e' proporzionale a 1/SNR (limite di Cramer-Rao del ToA); il primo e' il pavimento del chip.
// 3) Riconoscimento NLOS imperfetto (il chip lo stima dalla differenza tra potenza totale e del primo cammino):
//    in LOS ogni messaggio e' dichiarato NLOS con probabilita' pFa (indipendente per messaggio); un episodio NLOS
//    resta non rilevato per intero con probabilita' pMiss (bias persistente dichiarato LOS: il caso peggiore).
// Valori di default: IPOTESI documentate in PAPER_NOTES.md (sez. 4.26), da calibrare con misure reali.
#include "UwbLink.h"
#include <cmath>

namespace uwberr {

struct Params {
    bool   on          = false;
    double losDist     = 500.0;   // P_LOS(d) = exp(-d / losDist) (legge storica "outdoor")
    double corrDist    = 10.0;    // distanza di correlazione dello stato LOS [m] (3GPP: 10 m indoor, 50-60 m esterni)
    double sigmaFloor  = 0.10;    // rumore per messaggio con segnale forte [m] (storico; DW3000 +-10 cm)
    double sigmaEdge   = 0.25;    // rumore aggiuntivo per messaggio a margine 0 dB [m]
    double biasMin     = 0.3;     // bias NLOS per episodio ~ U(biasMin, biasMax) [m] (storico)
    double biasMax     = 2.5;
    double nlosJitter  = 0.5;     // rumore aggiuntivo per messaggio in NLOS [m] (storico)
    double pMiss       = 0.10;    // episodio NLOS non rilevato
    double pFa         = 0.05;    // messaggio LOS dichiarato NLOS
    uwblink::Params link;         // bilancio di collegamento (sensibilita' = --uwbSensDbm)
};

// Margine [dB] tra potenza media ricevuta e sensibilita' a distanza d
inline double MarginDb(const Params& p, double d) { return uwblink::MeanRxDbm(p.link, d) - p.link.sensDbm; }

// Varianza dell'errore di un messaggio in LOS [m^2]
inline double MsgVarLos(const Params& p, double d) {
    return p.sigmaFloor * p.sigmaFloor + p.sigmaEdge * p.sigmaEdge * std::pow(10.0, -MarginDb(p, d) / 10.0);
}

// Secondo momento del bias NLOS per episodio, U(a, b): (a^2 + ab + b^2) / 3
inline double BiasSecondMoment(const Params& p) {
    return (p.biasMin * p.biasMin + p.biasMin * p.biasMax + p.biasMax * p.biasMax) / 3.0;
}

// Varianza da usare nei filtri per una distanza DS-TWR, dato il bit LOS dichiarato e la distanza misurata.
// DS-TWR: errore = w1 e1 + w2 e2 + w3 e3 con w2 = 1/2 e w1 + w3 = 1/2, quindi varianza tra 3/8 e 1/2 di quella
// per messaggio; si usa quella per messaggio (margine 2x, come il valore storico 0,01 m^2 con sigma 0,10 m).
// NLOS: il bias e' persistente e non noto, quindi entra come secondo momento (E[b^2] + jitter^2 + rumore).
inline double DsTwrVar(const Params& p, bool los, double d) {
    double v = MsgVarLos(p, d);
    return los ? v : v + p.nlosJitter * p.nlosJitter + BiasSecondMoment(p);
}

// Funzione di ripartizione della normale standard
inline double Phi(double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); }

}  // namespace uwberr
