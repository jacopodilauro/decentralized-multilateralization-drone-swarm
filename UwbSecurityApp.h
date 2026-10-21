#ifndef UWB_SECURITY_APP_H
#define UWB_SECURITY_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/mobility-model.h"
#include "ns3/event-id.h"
#include "UWBChannel.h"
#include "UwbHeader.h"
#include "EKF.h"
#include "ClockModel.h"
#include "GnssModel.h"
#include "ImuModel.h"
#include "UwbLink.h"
#include "random"

#include <Eigen/Dense>
#include <map>
#include <algorithm>
#include <vector>
#include <fstream>
#include <set>
#include <deque>
#include <set>
using namespace ns3;

class UwbSecurityApp : public Application {
public:
    
    enum MacState {
        STATE_OUT_OF_RANGE,
        STATE_LISTENING,
        STATE_JOINING,
        STATE_ACTIVE
    };

    void SetNodeRole(bool isGuest);

    static TypeId GetTypeId(void);

    // Livello di output a terminale: 0 = silenzioso, 1 = eventi (default), 2 = debug completo
    static void SetVerbosity(uint32_t level);
    // Ranging: false = ToA a una via (orologi quasi sincronizzati, modello storico),
    //          true  = DS-TWR dai broadcast TDMA con orologi UWB realistici
    static void SetRangingDsTwr(bool on);
    // GNSS: con una costellazione -> ricevitore simulato (pseudodistanze + minimi quadrati);
    //       con nullptr -> modello storico (posizione vera + rumore bianco)
    static void SetGnssConstellation(std::shared_ptr<gnss::Constellation> c);
    // Spoofer GNSS esterno (Fase 7): cattura il ricevitore di ogni drone da spoofer->startS.
    // Con logPath non vuoto ogni fix GPS viene scritto in un CSV (istante, nodo, catturato, GPS, verita', PPS)
    static void SetGnssSpoofer(std::shared_ptr<const gnss::Spoofer> s, const std::string& logPath);
    // Soglia del test chi-quadro (3 gradi di liberta') usata con il ricevitore GNSS
    static void SetChi2Threshold(double t) { s_chi2Thr = t; }
    // Range condivisi per pacchetto: K e politica di scelta.
    //   nearest: i K piu' vicini (storico). Con sciami grandi i droni periferici non sono tra i K piu'
    //            vicini di nessuno: ricevono solo il range diretto e il loro filtro diverge
    //   oldest:  i K condivisi meno di recente (rotazione): ogni range esce ogni ceil(N/K) pacchetti
    static void SetRangeSharing(uint32_t k, bool oldestFirst) { s_shareK = k; s_shareOldest = oldestFirst; }
    // Incertezza GPS delle ancore (solo con il ricevitore GNSS):
    //   0 fixed:    R fisso, test con S = P + 2 Sigma_mittente (6.2 iniziale)
    //   1 r:        R_i += u' Sigma_ancora u, test con S = P + Sigma_mittente
    //   2 consider: come r, e nel test anche la covarianza del bias dovuto alle ancore (lento)
    static void SetAnchorModel(int m) { s_anchorModel = m; }
    // Sensori di bordo indipendenti dal GPS (8.2c): sotto cattura ogni drone misura e annuncia la propria velocita'
    static void SetImu(bool on, const imu::Params& p) { s_imuOn = on; s_imuParams = p; }
    // Fase 3: radio UWB (pacchetti persi in funzione della distanza, UwbLink.h) invece del Wi-Fi in cui tutti
    // ricevono tutto. Il trasporto resta lo stack Wi-Fi di ns-3: con TDMA e slot di 5 ms la durata dei pacchetti
    // (~0,5 ms a 6,8 Mbit/s con preambolo) non conta; qui si decide solo SE il pacchetto arriva
    static void SetUwbRadio(bool on, const uwblink::Params& p) { s_uwbRadio = on; s_linkParams = p; }
    static void PrintLinkStats();
    // Fase 5: log di ogni DS-TWR (distanza vera al messaggio centrale, misurata, stato vero e dichiarato)
    static void SetUwbLog(const std::string& path);
    // PPS del ricevitore GNSS (9.1): dente di sega uniforme +-sawtooth (quantizzazione, compensabile con qErr) +
    // rumore gaussiano residuo; log delle differenze di PPS per osservatore (pps_log.csv)
    static void SetPps(double sawtoothNs, double noiseNs, const std::string& logPath);
    static void PrintPpsStats();
    double GetPpsTrueErr(uint8_t sec) const { auto it = m_ownPps.find(sec); return it == m_ownPps.end() ? NAN : it->second.trueErr; }
    // Eta' massima di un range per essere usato (e quindi condiviso)
    double MaxRangeAge() const { return std::min(1.0, 8.0 * m_swarmSize * m_slotDuration); }

    void SetActive(bool active);
    void ScheduleLeave();
    void RemovePeer(uint32_t peerId);
    void InitSlotMap(const std::vector<uint32_t>& activeIds);

    UwbSecurityApp();
    virtual ~UwbSecurityApp();

    void Setup(uint32_t id, uint32_t swarmSize, double slotDuration,
               Ptr<UWBChannel> channel, std::ofstream* csv);

    void SetMalicious(bool isMalicious);
    bool IsMalicious() const;
    
    double GetClockOffset() const { return m_clockOffset; }



private:

    MacState m_macState     = STATE_ACTIVE;
    bool     m_isGuest      = false;
    uint32_t m_listenCounter = 0;

    void EvaluateMacState();
    EventId m_macStateEvent;

    bool m_isActive   = true;
    bool m_imLeaving  = false;
    bool m_pendingLeave = false;

    virtual void StartApplication(void) override;
    virtual void StopApplication(void)  override;

    double m_clockOffset = 0.0;
    std::mt19937 m_rng;

    void SendUwbMessage();
    void ReceivePacket(Ptr<Socket> socket);
    void ProcessRanging(uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec,
                        const UwbHeader& header);
    void ReorganizeSlots(uint32_t leavingDroneId);

    Eigen::Vector3d GetCurrentGpsPosition();

    uint32_t        m_id;
    uint32_t        m_slotId;
    uint32_t        m_swarmSize;
    bool            m_isMalicious;
    double          m_attackStartTime;
    double          m_slotDuration;
    uint16_t        m_port;
    Ptr<UWBChannel> m_channel;
    std::ofstream*  m_csv;

    std::map<uint32_t, uint32_t> m_slotMap;

    Ptr<Socket> m_socket;
    EventId     m_sendEvent;

    std::vector<double> m_myLastRanges;
    std::vector<bool>   m_myLastRangesLos;
    std::vector<double> m_myLastRangeTime;   // istante in cui ho misurato ciascun range

    std::map<uint32_t, Eigen::Vector3d> m_lastKnownGps;
    std::map<uint32_t, double>          m_lastKnownTime;
    std::map<uint32_t, Eigen::Vector3d> m_lastKnownVelocity;

    std::map<uint32_t, double> m_ekfInitTime;

    std::map<uint32_t, std::map<uint32_t, double>> m_networkRanges;
    std::map<uint32_t, std::map<uint32_t, double>> m_networkRangeTimes;
    std::map<uint32_t, std::map<uint32_t, bool>>   m_networkRangesLos;

    std::map<uint32_t, EKF>    m_ekfBank;
    std::map<uint32_t, double> m_lastCalcTime;

    std::map<uint32_t, bool> m_alarms;
    std::map<uint32_t, int>  m_alarmCounter;
    std::map<uint32_t, std::set<uint32_t>> m_peerVotes;
    std::map<uint32_t, int>  m_okCounter;

    // Mappa: ID del drone che ha chiesto il Leave -> Elenco (Set) di chi ha approvato
    std::map<uint32_t, std::set<uint32_t>> m_pendingLeaves;
    // Mappa: ID del drone sospettato "Morto" -> Elenco (Set) di chi vota per cacciarlo
    // Voti di espulsione: drone -> (votante -> istante del voto). I voti scadono (PruneEvictionVotes) e un drone
    // vota contro X solo se non ha notizie di X da troppo tempo, ne' dirette ne' indirette (LastEvidence): con una
    // radio a portata limitata "io non lo sento" non significa "non c'e' piu'"
    std::map<uint32_t, std::map<uint32_t, double>> m_pendingEvictions;
    double LastEvidence(uint32_t id) const;
    void   PruneEvictionVotes(double now);
    // Lista dei droni che IO considero inattivi (da comunicare agli altri)
    std::set<uint32_t> m_myEvictionVotes;
    // Lista dei droni che IO so voler uscire (da comunicare agli altri)
    std::set<uint32_t> m_myLeaveVotes; 

    // --- Solo per l'output a terminale (non influenzano la simulazione) ---
    static uint32_t s_verbosity;
    // Evita che lo stesso evento venga stampato da ogni drone: true solo la prima volta in 1 s
    static bool AnnounceOnce(char kind, uint32_t droneId, double now);
    static std::map<std::pair<char, uint32_t>, double> s_lastAnnounce;
    // --- DS-TWR ---
    static bool s_rangingDsTwr;
    static std::shared_ptr<gnss::Constellation> s_constellation;
    static std::shared_ptr<const gnss::Spoofer> s_spoofer;
    static std::ofstream s_gnssLog;
    static double s_chi2Thr;
    static uint32_t s_shareK;
    static bool     s_shareOldest;
    static int      s_anchorModel;
    static bool         s_imuOn;
    static bool             s_uwbRadio;
    static uwblink::Params  s_linkParams;
    static uint64_t         s_linkTot[6], s_linkOk[6];   // per fasce di distanza di 30 m (l'ultima: oltre 150 m)
    std::mt19937_64         m_linkRng{ 11 };
    static imu::Params  s_imuParams;
    imu::Sensor     m_imu;
    std::map<uint32_t, std::pair<Eigen::Vector3d, double>> m_peerImuVel;   // (velocita' annunciata, istante)
    // 9.1: PPS marcato con l'orologio UWB; relazione tra il mio orologio e quello di ogni mittente
    static double s_ppsSawtoothS, s_ppsNoiseS;
    static std::ofstream s_ppsLog;
    static uint64_t s_ppsRejected[3];   // [1] differenze di PPS scartate per residuo della retta, [2] calcolate
    static std::ofstream s_uwbLog;
    double DsTwrVar(bool los, double range) const;
    struct OwnPps { uint64_t stamp; double trueErr; double t; };
    std::map<uint8_t, OwnPps> m_ownPps;        // per secondo (mod 256)
    int      m_lastPpsSec = -1;
    EventId  m_ppsEvent;
    std::mt19937_64 m_ppsRng{ 7 };
    void     OnPps();
    std::map<uint32_t, std::deque<std::pair<uint64_t, uint64_t>>> m_clockPairs;   // (tx mittente, rx mio - volo)
    std::map<uint32_t, std::deque<double>> m_clockPairTimes;   // istante di ricezione di ogni coppia [s]
    std::map<uint32_t, int> m_lastPpsFrom;    // ultimo secondo di PPS elaborato per mittente
    std::vector<double> m_lastSharedTime;    // ultima volta che ho condiviso il range verso ciascun drone
    std::map<uint32_t, Eigen::Matrix3d> m_lastKnownGpsCov;   // covarianza GPS individuale dichiarata [m^2]
    // Cattura dello sciame (Fase 8): frazione di coppie di altri droni la cui distanza dai GPS dichiarati e'
    // coerente con il range UWB. Uno spoofer a singola antenna fa vedere a tutti lo stesso punto: quasi
    // nessuna coppia e' coerente. Pochi droni bugiardi rendono incoerenti solo le coppie che li coinvolgono
    void   EvaluateSwarmCapture();
    double m_pairsOkFrac   = 1.0;
    // Storia breve dei GPS dichiarati (istante, posizione) per una velocita' INDIPENDENTE dagli EKF: sotto cattura
    // gli EKF (ancorati a GPS falsi) divergono e la loro velocita' rende "coerenti" coppie a caso
    std::map<uint32_t, std::deque<std::pair<double, Eigen::Vector3d>>> m_gpsHistory;
    Eigen::Vector3d ClaimedVelocity(uint32_t id) const;
    int    m_captureCount  = 0;      // frame consecutivi sospetti (> 0) o non sospetti (< 0)
    bool   m_swarmCaptured = false;
    // Consenso sulla cattura: ogni drone annuncia il proprio stato nel pacchetto; lo sciame e' catturato se almeno
    // 2/3 dei droni attivi (me compreso) lo annunciano. Sotto consenso: niente caccia al bugiardo e niente
    // aggiornamenti dei filtri con ancore GPS (8.2a: solo predizione; 8.2b: localizzazione relativa UWB)
    std::map<uint32_t, std::pair<bool, double>> m_peerCapture;   // (annuncio, istante)
    bool   m_captureConsensus = false;
    // 8.2b: localizzazione solo relativa sotto cattura. Istantanee dei filtri prese ogni secondo SOLO senza alcun
    // sospetto; al consenso si riparte dalla piu' vecchia (1-2 s prima dell'attacco). Ancore = mie stime degli
    // altri droni (non i loro GPS falsi); un filtro su me stesso fa da ancora per le misure dirette
    struct Snapshot {
        double t = -1.0;
        std::map<uint32_t, EKF> bank;
        std::map<uint32_t, double> lastCalc;
        Eigen::Vector3d myGps = Eigen::Vector3d::Zero(), myVel = Eigen::Vector3d::Zero();
        Eigen::Matrix3d myCov = Eigen::Matrix3d::Identity();
    };
    Snapshot m_snapOld, m_snapNew;
    bool   m_relMode = false;
    EKF    m_selfEkf;
    double m_selfTime = 0.0;
    Eigen::Vector3d m_myLastGps = Eigen::Vector3d::Zero();
    void   TakeSnapshot(double now);
    void   EnterRelativeMode(double now);
    Eigen::Vector3d SelfAt(double t, Eigen::Matrix3d* cov);   // mia posizione stimata all'istante t
    std::unique_ptr<gnss::Receiver> m_gnss;   // ricevitore GNSS di questo drone
    gnss::Fix m_lastGnssFix;                  // ultimo fix (il clock bias servira' per lo spoofer)
    uwbclock::Clock m_uwbClock;          // orologio del chip UWB di questo drone
    std::mt19937    m_clockRng;          // rumore dei timestamp (separato dal rumore GPS)
    uint8_t         m_txSeq = 0;
    struct TxRec { uint64_t stamp; double tGlobal; Eigen::Vector3d gps; };
    std::map<uint8_t, TxRec> m_txHistory;            // mie trasmissioni recenti, per numero di sequenza
    struct RxRec { uint8_t seq; uint64_t txStamp; uint64_t rxStamp; double txGlobal; double rxGlobal; bool los; bool losTrue; };
    std::map<uint32_t, RxRec> m_lastRx;              // ultimo pacchetto ricevuto da ciascun vicino

    std::map<uint32_t, bool> m_collectiveAlarm;   // mio allarme collettivo su ciascun drone
    std::map<uint32_t, uint32_t> m_lastPrintedVoteState;  // per stampare i cambi di voto (livello 2)
};

#endif