#ifndef SIMULATION_LOGGER_H
#define SIMULATION_LOGGER_H

#include <Eigen/Dense>
#include <fstream>

class SimulationLogger {
public:
    static void LogObservation(
        double time, 
        int sender_id, 
        int observer_id,
        Eigen::Vector3d estimated_pos,
        Eigen::Vector3d claimed_gps, 
        Eigen::Vector3d true_pos,
        bool alarm,
        Eigen::Vector3d recovered_pos,
        std::ofstream& csv,
        uint32_t totalVotes, 
        uint32_t threshold,
        uint32_t activeNodes,
        uint32_t peerVotes,
        double clockBias,
        double ekfStdDev, 
        double true_distance
    ) {
        double discrepancy = (estimated_pos - claimed_gps).norm();
        double estimation_error = (estimated_pos - true_pos).norm();

        csv << time << "," << sender_id << "," << observer_id << ","
            << estimated_pos.x() << "," << estimated_pos.y() << "," << estimated_pos.z() << ","
            << claimed_gps.x() << "," << claimed_gps.y() << "," << claimed_gps.z() << ","
            << true_pos.x() << "," << true_pos.y() << "," << true_pos.z() << ","
            << discrepancy << "," << estimation_error << "," << alarm << ","
            << recovered_pos.x() << "," << recovered_pos.y() << "," << recovered_pos.z() << ","
            << totalVotes << "," << threshold << "," 
            << activeNodes << "," << peerVotes << "," << clockBias << ","
            << ekfStdDev << "," << true_distance <<"\n";
    }
};

#endif