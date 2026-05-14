#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <vector>

class EKF {
public:
    static constexpr double c = 299792458.0;

    struct Msmnt {
        Eigen::Vector3d anchor_pos;
        double          toa; 
        double          tx_timestamp;
        bool            is_direct;   
        bool            is_los;      
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
    Eigen::VectorXd m_state;  
    Eigen::MatrixXd m_P;     
    Eigen::MatrixXd m_Q;     

    double m_q_acc   = 0.5;  
    double m_q_drift = 0.05;   

    //ultima innovazione per Mahalanobis
    Eigen::VectorXd m_last_y;
    Eigen::MatrixXd m_last_S;
};

#endif