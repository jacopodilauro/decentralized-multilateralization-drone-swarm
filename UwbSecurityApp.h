#ifndef UWB_SECURITY_APP_H
#define UWB_SECURITY_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/mobility-model.h"
#include "ns3/event-id.h"
#include "UWBChannel.h"
#include "UwbHeader.h"
#include "EKF.h"
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
    int32_t  m_chosenSlot   = -1;

    std::map<uint32_t, bool> m_localSlotMap;
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
    void ProcessRanging(uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec);
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
    std::map<uint32_t, std::set<uint32_t>> m_pendingEvictions;
    // Lista dei droni che IO considero inattivi (da comunicare agli altri)
    std::set<uint32_t> m_myEvictionVotes;
    // Lista dei droni che IO so voler uscire (da comunicare agli altri)
    std::set<uint32_t> m_myLeaveVotes; 

    // --- Solo per l'output a terminale (non influenzano la simulazione) ---
    static uint32_t s_verbosity;
    // Evita che lo stesso evento venga stampato da ogni drone: true solo la prima volta in 1 s
    static bool AnnounceOnce(char kind, uint32_t droneId, double now);
    static std::map<std::pair<char, uint32_t>, double> s_lastAnnounce;
    std::map<uint32_t, uint32_t> m_lastPrintedVoteState;  // per stampare i cambi di voto (livello 2)
};

#endif