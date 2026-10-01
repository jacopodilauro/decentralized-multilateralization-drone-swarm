#ifndef UWB_CHANNEL_H
#define UWB_CHANNEL_H

#include "ns3/core-module.h"
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace ns3;
using namespace Eigen;

struct ChannelCondition {
    bool is_los;              
    double path_loss_db;      
    double delay_spread_ns;   
    double rssi_dbm;          
    double ranging_error_m;   
};

class UWBChannel : public Object
{
public:
    UWBChannel();
    virtual ~UWBChannel();

    static TypeId GetTypeId();
    // txId/rxId identificano il collegamento: ogni collegamento ha il proprio generatore casuale,
    // cosi' il rumore di un collegamento non dipende da quante misure fanno gli altri
    ChannelCondition ComputeChannelCondition(uint32_t txId, uint32_t rxId,
                                             Vector3d tx_pos, Vector3d rx_pos, double tx_power_dbm = 0.0);
    
    void SetEnvironment(std::string env_type); 
    void AddObstacle(Vector3d center, double radius);
    void SetSeed(uint64_t seed); 
    
private:
    std::string m_environment;
    std::vector<std::pair<Vector3d, double>> m_obstacles; 
    uint64_t m_baseSeed;
    std::map<std::pair<uint32_t, uint32_t>, std::mt19937> m_linkRng;

    std::mt19937& LinkRng(uint32_t txId, uint32_t rxId);
    bool DetermineLOS(std::mt19937& rng, Vector3d tx, Vector3d rx);
    double ComputePathLoss(std::mt19937& rng, double distance_m, bool is_los);
    double ComputeDelaySpread(double distance_m, bool is_los);
    double ComputeRangingError(std::mt19937& rng, bool is_los, double distance_m);
};

#endif
