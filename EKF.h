#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <vector>

class EKF {
public:
    static constexpr double c = 299792458.0;

    struct Msmnt {
        Eigen::Vector3d anchor_pos;  // posizione dell'anchor
        double          toa;         // time of arrival [s]
        double          tx_timestamp;// timestamp [s]
        bool            is_direct;   // true = misura diretta, false = peer
        bool            is_los;      // true = LOS, false = NLOS
        double          range = 0.0;
        Msmnt() : toa(0), tx_timestamp(0), is_direct(true), is_los(true) {}
    };

    EKF();

    void Init(const Eigen::Vector3d& init_pos);
    void Predict(double dt);
    void Update(const std::vector<Msmnt>& measurements);

    Eigen::Vector3d GetPosition()        const;
    Eigen::VectorXd GetState()           const;
    Eigen::MatrixXd GetCovariance()      const;
    double          GetPositionStdDev()  const;

    // Soglia adattiva per allarme
    double GetMahalanobisDistance() const;

private:
    Eigen::VectorXd m_state;   // [x, y, z, vx, vy, vz, bt]
    Eigen::MatrixXd m_P;       // covarianza di stato
    Eigen::MatrixXd m_Q;       // rumore di processo

    double m_q_acc   = 0.5;    // [m²/s³] accelerazione
    double m_q_drift = 0.05;   // [m²/s]  drift del clock

    // Salvataggi ultima innovazione per Mahalanobis
    Eigen::VectorXd m_last_y;
    Eigen::MatrixXd m_last_S;
};

#endif