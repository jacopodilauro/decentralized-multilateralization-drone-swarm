#include "EKF.h"
#include <iostream>

using namespace Eigen;
using namespace std;

EKF::EKF() {
    m_state = VectorXd::Zero(7);
    m_P     = MatrixXd::Identity(7, 7);
    m_Q     = MatrixXd::Identity(7, 7);
}

void EKF::Init(const Vector3d& init_pos) {
    m_state = VectorXd::Zero(7);
    m_state.segment<3>(0) = init_pos;
    m_state(6) = 0.0;   

    m_P = MatrixXd::Identity(7, 7);
    m_P.block<3,3>(0,0) *= 10.0;  
    m_P.block<3,3>(3,3) *= 2.0;   
    m_P(6,6)             = 1000.0;  

    m_Q = MatrixXd::Zero(7, 7);
    /*m_q_acc   = 0.5; 
    m_q_drift = 0.01;*/
    m_q_acc   = 3; 
    m_q_drift = 0.1;
}

void EKF::Predict(double dt) {
    if (dt <= 0) return;

    MatrixXd F = MatrixXd::Identity(7, 7);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    MatrixXd Q = MatrixXd::Zero(7, 7);

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    for (int axis = 0; axis < 3; ++axis) {
        int p = axis;       
        int v = axis + 3;  
        Q(p, p) = m_q_acc * dt3 / 3.0;
        Q(p, v) = m_q_acc * dt2 / 2.0;
        Q(v, p) = m_q_acc * dt2 / 2.0;
        Q(v, v) = m_q_acc * dt;
    }

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
    MatrixXd R = MatrixXd::Zero(n, n);

    Vector3d est_pos  = m_state.segment<3>(0);
    double   est_bias = m_state(6);

    for (int i = 0; i < n; ++i) {
        const Msmnt& m = measurements[i];

        double geo_dist  = (est_pos - m.anchor_pos).norm();
        double safe_dist = geo_dist + 1e-9;

        H(i, 0) = (est_pos.x() - m.anchor_pos.x()) / safe_dist;
        H(i, 1) = (est_pos.y() - m.anchor_pos.y()) / safe_dist;
        H(i, 2) = (est_pos.z() - m.anchor_pos.z()) / safe_dist;
        H(i, 3) = 0.0; H(i, 4) = 0.0; H(i, 5) = 0.0;

        if (m.is_direct) {
            Z(i)    = (m.toa - m.tx_timestamp) * c;
            h(i)    = geo_dist + est_bias;
            H(i, 6) = 1.0;
            R(i, i) = m.is_los ? 0.09 : 0.36;
        } else {
            Z(i)    = m.range;
            h(i)    = geo_dist;
            H(i, 6) = 0.0;
            R(i, i) = m.is_los ? 1.5 : 4.0;
        }
    }

    VectorXd y = Z - h;

    MatrixXd S = H * m_P * H.transpose() + R;

    MatrixXd K = S.ldlt().solve(H * m_P).transpose();
    m_state = m_state + K * y;

    MatrixXd I_KH = MatrixXd::Identity(7, 7) - K * H;
    m_P = I_KH * m_P * I_KH.transpose() + K * R * K.transpose();

    m_last_y = y.head(1);
    m_last_S = S.block<1,1>(0,0);
}

double EKF::GetMahalanobisDistance() const {
    if (m_last_y.size() == 0) return 0.0;
    return std::sqrt(m_last_y.dot(m_last_S.ldlt().solve(m_last_y)));
}

Vector3d EKF::GetPosition()             const { return m_state.segment<3>(0); }
VectorXd EKF::GetState()               const { return m_state; }
MatrixXd EKF::GetCovariance()          const { return m_P; }
double   EKF::GetPositionStdDev()      const {
    return std::sqrt((m_P(0,0) + m_P(1,1) + m_P(2,2)) / 3.0);
}
