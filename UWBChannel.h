#ifndef UWB_CHANNEL_H
#define UWB_CHANNEL_H

#include "ns3/core-module.h"
#include "UwbError.h"
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
    bool   los_flag;          // stato dichiarato dal ricevitore (rivelatore NLOS); storico: = is_los
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
    // Fase 5: errore UWB realistico (UwbError.h). Con p.on = false il modello storico resta identico
    void SetErrorModel(const uwberr::Params& p) { m_err = p; }
    const uwberr::Params& GetErrorModel() const { return m_err; }
    // Diagnostica: ultimo stato vero del collegamento tx -> rx (true = LOS) e statistiche a fine simulazione
    bool LastTrueLos(uint32_t txId, uint32_t rxId) const;
    void PrintErrorStats() const;
    
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

    // Fase 5: stato di ogni collegamento non orientato (a < b)
    struct LinkState {
        bool init = false;
        Vector3d pa, pb;          // posizioni degli estremi all'ultimo aggiornamento
        double sPath = 0.0;       // spostamento cumulato degli estremi (con fase casuale) [m]
        long   knot = 0;          // nodo corrente del processo: g0 a knot*L, g1 a (knot+1)*L
        double g0 = 0.0, g1 = 0.0;
        bool nlos = false;
        double bias = 0.0;        // bias NLOS dell'episodio [m]
        bool hidden = false;      // episodio non rilevato dal rivelatore NLOS
        double tStart = 0.0;
        std::mt19937 rng;
    };
    uwberr::Params m_err;
    std::map<std::pair<uint32_t, uint32_t>, LinkState> m_links;
    std::map<std::pair<uint32_t, uint32_t>, bool> m_lastTrueLos;
    // statistiche: messaggi [vero NLOS][dichiarato NLOS], episodi NLOS e loro durata
    uint64_t m_conf[2][2] = { { 0, 0 }, { 0, 0 } };
    uint64_t m_episodes = 0, m_hiddenEpisodes = 0;
    double m_episodeTime = 0.0;
    ChannelCondition ComputeReal(uint32_t txId, uint32_t rxId, Vector3d tx_pos, Vector3d rx_pos, double tx_power_dbm);
};

#endif
