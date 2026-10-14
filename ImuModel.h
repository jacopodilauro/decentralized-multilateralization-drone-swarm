#pragma once
// Sensore di velocita' di bordo indipendente dal GPS (8.2c): IMU + odometria (optical flow / visiva) + bussola.
// Misura la velocita' nel riferimento nord/est/su con:
//   - rumore bianco per asse (velNoise);
//   - bias lento per asse, Gauss-Markov (velBias, velBiasTau);
//   - errore di scala fisso per drone (scale, 1 sigma relativo);
//   - errore di direzione della bussola, Gauss-Markov (headingDeg, headingTau): ruota la velocita' orizzontale.
// Valori di default PRUDENTI dalla letteratura sull'optical flow (errori di qualche decimo di m/s, qualche %);
// l'errore della bussola e' un'IPOTESI da verificare. Tutti configurabili per l'analisi di sensibilita'.
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include "GnssModel.h"   // gnss::GaussMarkov

namespace imu {

struct Params {
    double velNoise   = 0.2;    // [m/s]
    double velBias    = 0.3;    // [m/s]
    double velBiasTau = 30.0;   // [s]
    double scale      = 0.03;   // relativo
    double headingDeg = 3.0;    // [gradi]
    double headingTau = 120.0;  // [s]
};

class Sensor {
public:
    Sensor() = default;
    Sensor(const Params& p, uint64_t seed) : m_p(p), m_rng(seed) {
        for (auto& b : m_bias) { b.sigma = p.velBias; b.tau = p.velBiasTau; }
        m_heading.sigma = p.headingDeg * M_PI / 180.0; m_heading.tau = p.headingTau;
        std::normal_distribution<double> n(0.0, 1.0);
        m_scale = 1.0 + p.scale * n(m_rng);
    }
    Eigen::Vector3d Measure(const Eigen::Vector3d& vTrue, double t) {
        std::normal_distribution<double> n(0.0, 1.0);
        double psi = m_heading.At(t, m_rng), c = std::cos(psi), s = std::sin(psi);
        Eigen::Vector3d v = m_scale * vTrue;
        v = Eigen::Vector3d(c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z());
        for (int i = 0; i < 3; i++) v(i) += m_bias[i].At(t, m_rng) + m_p.velNoise * n(m_rng);
        return v;
    }
    // Varianza per asse da usare nel filtro (tratta l'errore come bianco: approssimazione dichiarata)
    Eigen::Matrix3d Cov(const Eigen::Vector3d& vMeas) const {
        double h = m_p.headingDeg * M_PI / 180.0 * vMeas.head<2>().norm();
        double base = m_p.velNoise * m_p.velNoise + m_p.velBias * m_p.velBias;
        Eigen::Vector3d d(base + h * h + std::pow(m_p.scale * vMeas.x(), 2),
                          base + h * h + std::pow(m_p.scale * vMeas.y(), 2),
                          base + std::pow(m_p.scale * vMeas.z(), 2));
        return d.asDiagonal();
    }
    const Params& P() const { return m_p; }
private:
    Params m_p;
    std::mt19937_64 m_rng{ 1 };
    gnss::GaussMarkov m_bias[3], m_heading;
    double m_scale = 1.0;
};

}  // namespace imu
