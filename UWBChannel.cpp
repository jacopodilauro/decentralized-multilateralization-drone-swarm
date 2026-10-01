#include "UWBChannel.h"

#include "ns3/rng-seed-manager.h"

#include <random>
#include <cmath>

using namespace ns3;
using namespace Eigen;
using namespace std;
 
NS_OBJECT_ENSURE_REGISTERED (UWBChannel);

TypeId UWBChannel::GetTypeId()
{
    static TypeId tid = TypeId("UWBChannel").SetParent<Object>().SetGroupName("Custom").AddConstructor<UWBChannel>();
    return tid;
}

UWBChannel::UWBChannel() : m_environment("outdoor")
{
    m_baseSeed = RngSeedManager::GetSeed() * 6364136223846793005ULL + RngSeedManager::GetRun();
}

void UWBChannel::SetSeed(uint64_t seed)
{
    m_baseSeed = seed;
    m_linkRng.clear();
}

std::mt19937& UWBChannel::LinkRng(uint32_t txId, uint32_t rxId)
{
    auto key = std::make_pair(txId, rxId);
    auto it = m_linkRng.find(key);
    if (it == m_linkRng.end()) {
        std::seed_seq seq{ (uint32_t)(m_baseSeed & 0xFFFFFFFFu), (uint32_t)(m_baseSeed >> 32),
                           txId, rxId, 0xC4A11EEDu };
        it = m_linkRng.emplace(key, std::mt19937(seq)).first;
    }
    return it->second;
}

UWBChannel::~UWBChannel() {}

void UWBChannel::SetEnvironment(std::string env_type) { m_environment = env_type; }

void UWBChannel::AddObstacle(Vector3d center, double radius) {
    m_obstacles.push_back({center, radius});
}

bool UWBChannel::DetermineLOS(std::mt19937& rng, Vector3d tx, Vector3d rx)
{
    double distance = (rx - tx).norm();
    double p_los;
    
    if (m_environment == "outdoor") {
        p_los = std::exp(-distance / 500.0); 
    } else if (m_environment == "indoor") {
        p_los = std::exp(-distance / 30.0);
    } else {
        p_los = std::exp(-distance / 150.0);
    }
    
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    return uniform(rng) < p_los;
}

double UWBChannel::ComputePathLoss(std::mt19937& rng, double distance_m, bool is_los)
{
    double freq_ghz = 6.5; // Frequenza UWB tipica in GHz
    double fspl_db = 20 * std::log10(distance_m) + 
                     20 * std::log10(freq_ghz) + 
                     92.45;
    
    if (is_los) {
        std::normal_distribution<double> shadow_fading(0.0, 3.0);
        return fspl_db + shadow_fading(rng);
    } else {
        double excess_pl = 0.0;
        if (m_environment == "outdoor") {
            excess_pl = 5.0 + 10 * std::log10(distance_m / 10.0);
        } else {
            excess_pl = 10.0 + 15 * std::log10(distance_m / 10.0);
        }
        
        std::normal_distribution<double> shadow_fading(0.0, 6.0); 
        return fspl_db + excess_pl + shadow_fading(rng);
    }
}

double UWBChannel::ComputeDelaySpread(double distance_m, bool is_los) 
{
    if (is_los) {
        return 5.0 + distance_m * 0.02;
    } else {
        if (m_environment == "outdoor") {
            return 15.0 + distance_m * 0.1;
        } else {
            return 25.0 + distance_m * 0.3;
        }
    }
}

double UWBChannel::ComputeRangingError(std::mt19937& rng, bool is_los, double distance_m)
{
    double base_error;
    
    if (is_los) {
        std::normal_distribution<double> los_error(0.0, 0.10); 
        base_error = los_error(rng);
    } else {
        std::normal_distribution<double> nlos_variance(0.0, 0.50); 
        std::uniform_real_distribution<double> nlos_bias(0.3, 2.5); 
        
        base_error = nlos_bias(rng) + nlos_variance(rng);
    }
    
    double distance_factor = 1.0 + (distance_m / 200.0);
    return base_error * distance_factor;
}

ChannelCondition UWBChannel::ComputeChannelCondition(uint32_t txId, uint32_t rxId,
                                                     Vector3d tx_pos, Vector3d rx_pos, double tx_power_dbm) {
    ChannelCondition cond;
    double distance_m = (rx_pos - tx_pos).norm();
    std::mt19937& rng = LinkRng(txId, rxId);

    cond.is_los = DetermineLOS(rng, tx_pos, rx_pos);
    cond.path_loss_db = ComputePathLoss(rng, distance_m, cond.is_los);
    cond.rssi_dbm = tx_power_dbm - cond.path_loss_db;
    cond.delay_spread_ns = ComputeDelaySpread(distance_m, cond.is_los);
    cond.ranging_error_m = ComputeRangingError(rng, cond.is_los, distance_m);
    
    return cond;
}
