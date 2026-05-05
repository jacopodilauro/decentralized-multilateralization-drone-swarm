#include "EKF.h"
#include <iostream>

using namespace Eigen;
using namespace std;

// ---------------------------------------------------------------------------
// Stato EKF: [x, y, z, vx, vy, vz, b]  (7 dimensioni)
//
// b = bias di clock del target
//
// Modello di misura TOA con bias:
//   misura_diretta : ρ_i = ||p - a_i|| + b  + noise   (H(i,6) = 1.0)
//   misura_peer    : ρ_k = ||p - a_k|| + 0  + noise   (H(k,6) = 0.0)
//                    Il bias del peer non è osservabile → ignorato,
//                    ma R_kk è aumentato per riflettere l'incertezza aggiuntiva.
// ---------------------------------------------------------------------------

EKF::EKF() {
    m_state = VectorXd::Zero(7);
    m_P     = MatrixXd::Identity(7, 7);
    m_Q     = MatrixXd::Identity(7, 7);
}

void EKF::Init(const Vector3d& init_pos) {
    m_state = VectorXd::Zero(7);
    m_state.segment<3>(0) = init_pos;
    m_state(6) = 0.0;   // bias iniziale sconosciuto -> zero

    m_P = MatrixXd::Identity(7, 7);
    m_P.block<3,3>(0,0) *= 10.0;  // incertezza posizione iniziale [m²]
    m_P.block<3,3>(3,3) *= 2.0;   // incertezza velocità iniziale  [(m/s)²]
    m_P(6,6)             = 25.0;  // incertezza bias iniziale [m²]
                                   // (equivale a circa 83 ns di incertezza di clock)

    // Rumore di processo Q:
    // - posizione: propagata dalla velocità, non serve rumore diretto
    // - velocità:  accelerazione non modellata ~0.5 m/s² → Q_v = σ_a² * dt
    //              qui mettiamo un valore base, viene scalato in Predict()
    // - bias:      drift di clock UWB tipico ~1 ppm → ~0.3 m/s di deriva
    //              Q_b = σ_drift² * dt, valore base qui
    m_Q = MatrixXd::Zero(7, 7);
    // I valori effettivi vengono applicati in Predict() scalati per dt
    // Per ora definiamo le densità spettrali di potenza (PSD):
    m_q_acc   = 0.5;   // [m²/s³] PSD accelerazione non modellata
    m_q_drift = 0.05;  // [m²/s]  PSD drift di clock (≈ 0.1 m/s tipico per UWB)
}

void EKF::Predict(double dt) {
    if (dt <= 0) return;

    // Matrice di transizione: modello velocità costante
    MatrixXd F = MatrixXd::Identity(7, 7);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    // bias modellato come costante con rumore (random walk)

    // Rumore di processo continuo → discretizzato per dt
    // Modello DWPA (Discrete Wiener Process Acceleration) per posizione/velocità
    MatrixXd Q = MatrixXd::Zero(7, 7);

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    // Blocco posizione-velocità (accoppiato)
    // Q_pos = q_acc * dt³/3,  Q_pv = q_acc * dt²/2,  Q_vel = q_acc * dt
    for (int axis = 0; axis < 3; ++axis) {
        int p = axis;       // indice posizione (0,1,2)
        int v = axis + 3;   // indice velocità  (3,4,5)
        Q(p, p) = m_q_acc * dt3 / 3.0;
        Q(p, v) = m_q_acc * dt2 / 2.0;
        Q(v, p) = m_q_acc * dt2 / 2.0;
        Q(v, v) = m_q_acc * dt;
    }

    // Bias: random walk con PSD m_q_drift
    Q(6, 6) = m_q_drift * dt;

    m_state = F * m_state;
    m_P     = F * m_P * F.transpose() + Q;
}

void EKF::Update(const vector<Msmnt>& measurements) {
    if (measurements.empty()) return;

    int n = measurements.size();
    VectorXd Z(n);
    VectorXd h(n);
    MatrixXd H = MatrixXd::Zero(n, 7);
    MatrixXd R = MatrixXd::Zero(n, n);   // diagonale, costruita per misura

    Vector3d est_pos  = m_state.segment<3>(0);
    double   est_bias = m_state(6);

    for (int i = 0; i < n; ++i) {
        const Msmnt& m = measurements[i];

        // Distanza misurata (pseudorange)
        double pseudorange = (m.toa - m.tx_timestamp) * c;
        Z(i) = pseudorange;

        double geo_dist  = (est_pos - m.anchor_pos).norm();
        double safe_dist = geo_dist + 1e-9;

        // Derivate parziali ∂||p-a||/∂p  (Jacobiana)
        H(i, 0) = (est_pos.x() - m.anchor_pos.x()) / safe_dist;
        H(i, 1) = (est_pos.y() - m.anchor_pos.y()) / safe_dist;
        H(i, 2) = (est_pos.z() - m.anchor_pos.z()) / safe_dist;
        H(i, 3) = 0.0; H(i, 4) = 0.0; H(i, 5) = 0.0;

        if (m.is_direct) {
            // --- MISURA DIRETTA: questo nodo è il ricevitore ---
            h(i)    = geo_dist + est_bias;
            H(i, 6) = 1.0;
            R(i, i) = m.is_los ? 0.0225 : 0.36;
        } else {
            // --- MISURA PEER: range osservato da un terzo nodo k ---
            h(i)    = geo_dist;   // no bias
            H(i, 6) = 0.0;        // bias non osservabile
            R(i, i) = m.is_los ? 1.36 : 2.00;
        }

        Z(i) = pseudorange;
        h(i) += 0.0;
    }

    VectorXd y = Z - h;

    // covarianza
    MatrixXd S = H * m_P * H.transpose() + R;

    MatrixXd K = m_P * H.transpose() * S.ldlt().solve(MatrixXd::Identity(n, n));

    m_state = m_state + K * y;

    // Aggiornamento covarianza — forma Joseph per stabilità numerica
    MatrixXd I_KH = MatrixXd::Identity(7, 7) - K * H;
    m_P = I_KH * m_P * I_KH.transpose() + K * R * K.transpose();

    // Salviamo S per la Mahalanobis distance (usata per l'allarme)
    m_last_S = S;
    m_last_y = y;
}

// ---------------------------------------------------------------------------
// Distanza di Mahalanobis dell'ultima innovazione
// Usata come soglia adattiva per l'allarme spoofing
// ---------------------------------------------------------------------------
double EKF::GetMahalanobisDistance() const {
    if (m_last_y.size() == 0) return 0.0;
    // y^T * S^{-1} * y
    return std::sqrt(m_last_y.dot(m_last_S.ldlt().solve(m_last_y)));
}

Vector3d EKF::GetPosition()             const { return m_state.segment<3>(0); }
VectorXd EKF::GetState()               const { return m_state; }
MatrixXd EKF::GetCovariance()          const { return m_P; }
double   EKF::GetPositionStdDev()      const {
    // deviazione standard posizione stimata == (media delle 3 componenti)
    return std::sqrt((m_P(0,0) + m_P(1,1) + m_P(2,2)) / 3.0);
}