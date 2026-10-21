#include "UWBChannel.h"

#include "ns3/rng-seed-manager.h"

#include <random>
#include <cmath>
#include <iomanip>
#include <iostream>

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
    if (m_err.on) return ComputeReal(txId, rxId, tx_pos, rx_pos, tx_power_dbm);
    ChannelCondition cond;
    double distance_m = (rx_pos - tx_pos).norm();
    std::mt19937& rng = LinkRng(txId, rxId);

    cond.is_los = DetermineLOS(rng, tx_pos, rx_pos);
    cond.path_loss_db = ComputePathLoss(rng, distance_m, cond.is_los);
    cond.rssi_dbm = tx_power_dbm - cond.path_loss_db;
    cond.delay_spread_ns = ComputeDelaySpread(distance_m, cond.is_los);
    cond.ranging_error_m = ComputeRangingError(rng, cond.is_los, distance_m);
    cond.los_flag = cond.is_los;
    
    return cond;
}

// ---------------------------------------------------------------------------
// Fase 5: modello realistico (vedi UwbError.h)
// ---------------------------------------------------------------------------
ChannelCondition UWBChannel::ComputeReal(uint32_t txId, uint32_t rxId, Vector3d tx_pos, Vector3d rx_pos,
                                         double tx_power_dbm)
{
    const uwberr::Params& p = m_err;
    double d = (rx_pos - tx_pos).norm();
    double now = Simulator::Now().GetSeconds();

    // 1) stato del collegamento non orientato: stesso processo nei due versi
    uint32_t a = std::min(txId, rxId), b = std::max(txId, rxId);
    Vector3d pa = (txId == a) ? tx_pos : rx_pos, pb = (txId == a) ? rx_pos : tx_pos;
    LinkState& st = m_links[{ a, b }];
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    // Processo gaussiano nello spostamento s: nodi indipendenti ogni L metri, interpolati linearmente e
    // rinormalizzati (marginale N(0,1) esatta). E' continuo: il numero di episodi non dipende da quanto spesso
    // si campiona (un Gauss-Markov del primo ordine invece attraversa la soglia sempre piu' spesso a passi piccoli).
    // L = corrDist / 0.905: la correlazione scende a 1/e a corrDist (come la correlazione esponenziale di 3GPP)
    const double L = p.corrDist / 0.905;
    if (!st.init) {
        std::seed_seq seq{ (uint32_t)(m_baseSeed & 0xFFFFFFFFu), (uint32_t)(m_baseSeed >> 32), a, b, 0x5A7E1105u };
        st.rng.seed(seq);
        st.sPath = (p.corrDist > 0.0) ? L * u01(st.rng) : 0.0;   // fase casuale rispetto ai nodi
        st.knot = 0;
        st.g0 = n01(st.rng); st.g1 = n01(st.rng);
        st.init = true;
    } else {
        st.sPath += (pa - st.pa).norm() + (pb - st.pb).norm();
    }
    double x;
    if (p.corrDist > 0.0) {
        long k = (long)std::floor(st.sPath / L);
        if (k == st.knot + 1) { st.g0 = st.g1; st.g1 = n01(st.rng); }
        else if (k > st.knot + 1) { st.g0 = n01(st.rng); st.g1 = n01(st.rng); }
        st.knot = k;
        double f = st.sPath / L - (double)k;
        x = ((1.0 - f) * st.g0 + f * st.g1) / std::sqrt((1.0 - f) * (1.0 - f) + f * f);
    } else {
        x = n01(st.rng);                   // senza correlazione: stato indipendente a ogni messaggio
    }
    st.pa = pa; st.pb = pb;
    bool los = uwberr::Phi(x) < std::exp(-d / p.losDist);
    if (!los && !st.nlos) {              // inizio di un episodio NLOS: bias e rilevabilita' per tutto l'episodio
        st.bias = p.biasMin + (p.biasMax - p.biasMin) * u01(st.rng);
        st.hidden = u01(st.rng) < p.pMiss;
        st.tStart = now;
        m_episodes++; if (st.hidden) m_hiddenEpisodes++;
    } else if (los && st.nlos) {
        m_episodeTime += now - st.tStart;
    }
    st.nlos = !los;

    // 2) errore del messaggio e 3) stato dichiarato, dal generatore del collegamento orientato
    std::mt19937& rng = LinkRng(txId, rxId);
    double var = uwberr::MsgVarLos(p, d);
    ChannelCondition cond;
    cond.is_los = los;
    if (los) {
        cond.ranging_error_m = std::sqrt(var) * n01(rng);
        cond.los_flag = !(u01(rng) < p.pFa);
    } else {
        cond.ranging_error_m = st.bias + std::sqrt(var + p.nlosJitter * p.nlosJitter) * n01(rng);
        cond.los_flag = st.hidden;
    }
    cond.rssi_dbm = uwblink::MeanRxDbm(p.link, d);
    cond.path_loss_db = tx_power_dbm - cond.rssi_dbm;
    cond.delay_spread_ns = ComputeDelaySpread(d, los);
    m_lastTrueLos[{ txId, rxId }] = los;
    m_conf[los ? 0 : 1][cond.los_flag ? 0 : 1]++;
    return cond;
}

bool UWBChannel::LastTrueLos(uint32_t txId, uint32_t rxId) const
{
    auto it = m_lastTrueLos.find({ txId, rxId });
    return it == m_lastTrueLos.end() ? true : it->second;
}

void UWBChannel::PrintErrorStats() const
{
    if (!m_err.on) return;
    uint64_t tot = m_conf[0][0] + m_conf[0][1] + m_conf[1][0] + m_conf[1][1];
    uint64_t nl = m_conf[1][0] + m_conf[1][1], lo = m_conf[0][0] + m_conf[0][1];
    double now = Simulator::Now().GetSeconds(), open = 0.0;
    for (const auto& kv : m_links) if (kv.second.init && kv.second.nlos) open += now - kv.second.tStart;
    std::cout << ">>> Errore UWB (Fase 5): messaggi " << tot << ", NLOS veri " << std::fixed << std::setprecision(2)
              << (tot ? 100.0 * nl / tot : 0.0) << "%, episodi NLOS " << m_episodes << " (non rilevati "
              << m_hiddenEpisodes << "), durata media " << (m_episodes ? (m_episodeTime + open) / m_episodes : 0.0)
              << " s | dichiarati NLOS: tra i LOS " << (lo ? 100.0 * m_conf[0][1] / lo : 0.0) << "%, tra gli NLOS "
              << (nl ? 100.0 * m_conf[1][1] / nl : 0.0) << "%" << std::defaultfloat << std::endl;
}
