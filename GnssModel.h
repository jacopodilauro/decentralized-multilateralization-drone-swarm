#ifndef GNSS_MODEL_H
#define GNSS_MODEL_H

// Modello di ricevitore GNSS a singola frequenza (nessuna dipendenza da ns-3).
//
// Pseudodistanza dal satellite j:   rho_j = |p - S_j| + c*b + E_j(t) + e_ij(t) + n_ij
//   E_j   errore COMUNE del satellite (ionosfera, troposfera, orbita, orologio del satellite):
//         uguale per tutti i ricevitori vicini, Gauss-Markov lento
//   e_ij  multipath del ricevitore i verso il satellite j: individuale, Gauss-Markov piu' veloce
//   n_ij  rumore di codice: individuale, bianco
//   b     clock bias del ricevitore (offset + deriva)
// Il ricevitore risolve posizione e clock bias ai minimi quadrati (Gauss-Newton), come uno vero.
// Coordinate locali ENU in metri; i satelliti sono fermi (in 240 s la loro direzione cambia di ~0.1 gradi).

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

namespace gnss {

constexpr double LIGHT_C = 299792458.0;

struct Params {
    int    numSats        = 9;       // satelliti visibili
    double elevMaskDeg    = 15.0;
    double satRangeM      = 22.0e6;  // distanza tipica di un satellite GPS
    // Valori tipici per un ricevitore consumer a cielo aperto (IPOTESI da giustificare con le fonti e da
    // variare nell'analisi di sensibilita'): orizzontale assoluto ~2-3 m, relativo tra droni vicini <1 m
    double commonSigmaM   = 1.5;     // errore comune per satellite (dev. std)
    double commonTauS     = 600.0;
    double multipathSigM  = 0.4;     // multipath individuale
    double multipathTauS  = 10.0;
    double codeNoiseSigM  = 0.2;     // rumore di codice individuale (bianco, pseudodistanze filtrate)
    double clockOffsetMaxS = 1e-3;   // offset iniziale dell'orologio del ricevitore: uniforme +-1 ms
    double clockDriftPpm  = 0.5;     // deriva tipica di un TCXO: uniforme +-0.5 ppm
};

// Processo di Gauss-Markov del primo ordine aggiornato a intervalli arbitrari
struct GaussMarkov {
    double x = 0.0, sigma = 0.0, tau = 1.0, t = 0.0;
    bool   init = false;
    template <class RNG> double At(double tNow, RNG& rng) {
        std::normal_distribution<double> n(0.0, 1.0);
        if (!init) { x = sigma * n(rng); t = tNow; init = true; return x; }
        double dt = tNow - t;
        if (dt > 0) {
            double a = std::exp(-dt / tau);
            x = a * x + sigma * std::sqrt(1.0 - a * a) * n(rng);
            t = tNow;
        }
        return x;
    }
};

// Costellazione condivisa: posizioni dei satelliti ed errori comuni (uguali per tutti i ricevitori)
class Constellation {
public:
    Constellation(const Params& p, uint64_t seed) : m_p(p), m_rng(seed) {
        std::uniform_real_distribution<double> u(0.0, 1.0);
        for (int j = 0; j < p.numSats; j++) {
            // azimut distribuiti, elevazioni tra la maschera e 85 gradi (geometria tipica, HDOP ~1)
            double az = 2.0 * M_PI * (j + 0.5 * u(m_rng)) / p.numSats;
            double el = (p.elevMaskDeg + (85.0 - p.elevMaskDeg) * u(m_rng)) * M_PI / 180.0;
            m_sats.emplace_back(p.satRangeM * std::cos(el) * std::sin(az),
                                p.satRangeM * std::cos(el) * std::cos(az),
                                p.satRangeM * std::sin(el));
            GaussMarkov g; g.sigma = p.commonSigmaM; g.tau = p.commonTauS;
            m_common.push_back(g);
        }
    }
    const std::vector<Eigen::Vector3d>& Sats() const { return m_sats; }
    // Errore comune del satellite j all'istante t (gli istanti devono essere non decrescenti)
    double CommonError(int j, double t) { return m_common[j].At(t, m_rng); }
    const Params& P() const { return m_p; }
private:
    Params m_p;
    std::mt19937_64 m_rng;
    std::vector<Eigen::Vector3d> m_sats;
    std::vector<GaussMarkov> m_common;
};

struct Fix {
    bool            ok = false;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    double          clockBiasS = 0.0;      // clock bias stimato [s]
    double          trueClockBiasS = 0.0;  // clock bias vero (solo per verifiche)
    double          residualRmsM = 0.0;
    int             numSats = 0;
    // Accuratezza INDIVIDUALE (1 sigma) dichiarata dal ricevitore: solo multipath + rumore di codice,
    // proiettati con la geometria dei satelliti. L'errore comune non c'e': si cancella tra droni vicini
    double          hAccM = 0.0;           // per asse orizzontale
    double          vAccM = 0.0;           // verticale
};

class Receiver {
public:
    Receiver() = default;
    Receiver(std::shared_ptr<Constellation> c, uint64_t seed) : m_c(std::move(c)), m_rng(seed) {
        const Params& p = m_c->P();
        std::uniform_real_distribution<double> off(-p.clockOffsetMaxS, p.clockOffsetMaxS);
        std::uniform_real_distribution<double> dr(-p.clockDriftPpm, p.clockDriftPpm);
        m_clockOffset = off(m_rng);
        m_clockDrift  = dr(m_rng) * 1e-6;
        for (int j = 0; j < p.numSats; j++) {
            GaussMarkov g; g.sigma = p.multipathSigM; g.tau = p.multipathTauS;
            m_multipath.push_back(g);
        }
    }

    double TrueClockBias(double t) const { return m_clockOffset + m_clockDrift * t; }

    // Pseudodistanze misurate all'istante t dalla posizione vera pTrue
    std::vector<double> Pseudoranges(const Eigen::Vector3d& pTrue, double t) {
        const Params& p = m_c->P();
        std::normal_distribution<double> n(0.0, p.codeNoiseSigM);
        std::vector<double> rho;
        double b = TrueClockBias(t);
        for (size_t j = 0; j < m_c->Sats().size(); j++) {
            double geo = (pTrue - m_c->Sats()[j]).norm();
            rho.push_back(geo + LIGHT_C * b + m_c->CommonError((int)j, t) + m_multipath[j].At(t, m_rng) + n(m_rng));
        }
        return rho;
    }

    // Soluzione ai minimi quadrati di posizione e clock bias (Gauss-Newton)
    Fix Solve(const std::vector<double>& rho) {
        Fix f;
        const auto& S = m_c->Sats();
        int n = (int)rho.size();
        f.numSats = n;
        if (n < 4) return f;
        Eigen::Vector4d x; x << m_lastPos, m_lastBiasM;     // parto dalla soluzione precedente
        for (int it = 0; it < 10; it++) {
            Eigen::MatrixXd H(n, 4); Eigen::VectorXd r(n);
            for (int j = 0; j < n; j++) {
                Eigen::Vector3d d = x.head<3>() - S[j];
                double geo = d.norm();
                H.block<1, 3>(j, 0) = (d / geo).transpose();
                H(j, 3) = 1.0;
                r(j) = rho[j] - (geo + x(3));
            }
            Eigen::Vector4d dx = (H.transpose() * H).ldlt().solve(H.transpose() * r);
            x += dx;
            if (dx.head<3>().norm() < 1e-4) break;
        }
        double ss = 0;
        Eigen::MatrixXd Hs(n, 4);
        for (int j = 0; j < n; j++) {
            Eigen::Vector3d d = x.head<3>() - S[j];
            double e = rho[j] - (d.norm() + x(3)); ss += e * e;
            Hs.block<1, 3>(j, 0) = (d / d.norm()).transpose(); Hs(j, 3) = 1.0;
        }
        Eigen::Matrix4d Gm = (Hs.transpose() * Hs).inverse();          // matrice DOP
        const Params& p = m_c->P();
        double sigInd2 = p.multipathSigM * p.multipathSigM + p.codeNoiseSigM * p.codeNoiseSigM;
        f.hAccM = std::sqrt(sigInd2 * 0.5 * (Gm(0, 0) + Gm(1, 1)));
        f.vAccM = std::sqrt(sigInd2 * Gm(2, 2));
        f.ok = true;
        f.pos = x.head<3>();
        f.clockBiasS = x(3) / LIGHT_C;
        f.residualRmsM = std::sqrt(ss / n);
        m_lastPos = f.pos; m_lastBiasM = x(3);
        return f;
    }

    // Misura e soluzione insieme
    Fix Measure(const Eigen::Vector3d& pTrue, double t) {
        Fix f = Solve(Pseudoranges(pTrue, t));
        f.trueClockBiasS = TrueClockBias(t);
        return f;
    }

private:
    std::shared_ptr<Constellation> m_c;
    std::mt19937_64 m_rng;
    double m_clockOffset = 0.0, m_clockDrift = 0.0;
    std::vector<GaussMarkov> m_multipath;
    Eigen::Vector3d m_lastPos = Eigen::Vector3d::Zero();
    double m_lastBiasM = 0.0;
};

} // namespace gnss

#endif
