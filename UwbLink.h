#pragma once
// Collegamento radio UWB (Fase 3): se un pacchetto arriva, in funzione della distanza.
//   potenza emessa   -41,3 dBm/MHz (limite FCC/ETSI) su 499,2 MHz (canale 5) = -14,3 dBm EIRP
//   attenuazione     spazio libero a 1 m alla frequenza del canale + 10 n log10(d) (IEEE 802.15.4a CM5, outdoor
//                    LOS: n = 1,76, ombreggiamento 0,83 dB) + 3 dB di perdita d'antenna
//   ricezione        probabilita' a S centrata sulla sensibilita' (default -100 dBm a 6,8 Mbit/s, da verificare nella
//                    sez. 3.4 del datasheet DW3000: dipende da preambolo e PRF), larghezza 1 dB
// Con i default un pacchetto arriva con probabilita' 50% a ~85 m; con sensibilita' -90 dBm a ~23 m (portata riportata
// in pratica da utenti DWM3000: 20-30 m). Il LOS/NLOS del collegamento e la correlazione dell'ombreggiamento sono della
// Fase 5: qui l'ombreggiamento e' estratto a ogni pacchetto.
#include <cmath>
#include <random>

namespace uwblink {

constexpr double LIGHT_C = 299792458.0;

struct Params {
    double psdDbmPerMhz = -41.3;      // limite di densita' spettrale (FCC/ETSI)
    double bandwidthMhz = 499.2;      // canale 5 e 9 HRP
    double freqHz       = 6.4896e9;   // canale 5
    double n            = 1.76;       // esponente outdoor LOS (802.15.4a CM5)
    double sigmaDb      = 0.83;       // ombreggiamento outdoor LOS (CM5)
    double antennaLossDb = 3.0;
    double sensDbm      = -100.0;
    double widthDb      = 1.0;
};

inline double TxEirpDbm(const Params& p) { return p.psdDbmPerMhz + 10.0 * std::log10(p.bandwidthMhz); }
inline double Pl0Db(const Params& p) { return 20.0 * std::log10(4.0 * M_PI * p.freqHz / LIGHT_C); }   // a 1 m
inline double MeanRxDbm(const Params& p, double d) {
    return TxEirpDbm(p) - Pl0Db(p) - 10.0 * p.n * std::log10(std::max(d, 1.0)) - p.antennaLossDb;
}
// distanza a cui la potenza media ricevuta e' pari alla sensibilita' (pacchetto ricevuto al 50% senza ombreggiamento)
inline double Range50(const Params& p) {
    return std::pow(10.0, (TxEirpDbm(p) - Pl0Db(p) - p.antennaLossDb - p.sensDbm) / (10.0 * p.n));
}
inline double Per(const Params& p, double rxDbm) { return 1.0 / (1.0 + std::exp((rxDbm - p.sensDbm) / p.widthDb)); }

// Estrae se il pacchetto arriva (ombreggiamento indipendente per pacchetto)
template <class RNG>
bool Received(const Params& p, double d, RNG& rng) {
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    double rx = MeanRxDbm(p, d) + p.sigmaDb * n01(rng);
    return u01(rng) >= Per(p, rx);
}

}  // namespace uwblink
