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

    void SetActive(bool active);
    void ScheduleLeave();
    void AddPeer(uint32_t peerId);
    void AddPeerSlot(uint32_t peerId, uint32_t slotId);
    void RemovePeer(uint32_t peerId);
    void InitSlotMap(const std::vector<uint32_t>& activeIds);

    UwbSecurityApp();
    virtual ~UwbSecurityApp();

    void Setup(uint32_t id, uint32_t swarmSize, double slotDuration,
               Ptr<UWBChannel> channel, std::ofstream* csv);

    void SetMalicious(bool isMalicious);
    bool IsMalicious() const;
    
    double GetClockOffset() const { return m_clockOffset; }

    uint32_t GetFirstAvailableSlot();
    bool JoinSwarm(uint32_t newDroneId);
    void ProcessIncomingPacket(uint32_t senderId, uint32_t slotId);
    void FinalizeJoin();

    std::map<uint32_t, uint32_t> GetSlotMap() const { return m_slotMap; }
    void SetSlotMap(const std::map<uint32_t, uint32_t>& newMap) { m_slotMap = newMap; }

        // Quanti peer hanno allarme attivo su questo nodo
    int GetActiveAlarmCount() const {
        int count = 0;
        for (const auto& pair : m_alarms)
            if (pair.second) count++;
        return count;
    }

    // Distanza di Mahalanobis media su tutti i peer tracciati
    double GetAverageMahalanobis() const {
        if (m_ekfBank.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& pair : m_ekfBank)
            sum += pair.second.GetMahalanobisDistance();
        return sum / m_ekfBank.size();
    }

    // Quanti nodi attivi vede questo nodo
    int GetActiveNodeCount() const {
        int count = 0;
        for (const auto& pair : m_slotMap)
            if (pair.second != UINT32_MAX) count++;
        return count;
    }

private:

    MacState m_macState;
    bool m_isGuest;
    uint32_t m_listenCounter;
    int32_t m_chosenSlot;

    std::map<uint32_t, bool> m_localSlotMap;
    std::set<uint32_t> m_localOccupiedSlots;
    void EvaluateMacState();
    EventId m_macStateEvent;

    bool m_isActive   = true;
    bool m_imLeaving  = false;
    bool m_pendingLeave = false;

    virtual void StartApplication(void) override;
    virtual void StopApplication(void)  override;

    double m_clockOffset;
    std::mt19937 m_rng;

    void SendUwbMessage();
    void ReceivePacket(Ptr<Socket> socket);
    void ProcessRanging(uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec);
    void ReorganizeSlots(uint32_t leavingDroneId);
    void PrintTerminalDashboard();

    Eigen::Vector3d GetCurrentGpsPosition();
    uint32_t        GetVoteBitmask();

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

    void RevertColor();

    std::vector<double> m_myLastRanges;
    std::vector<bool>   m_myLastRangesLos;

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

    std::deque<uint32_t> m_recentLeaves;
    // Mappa: ID del drone che ha chiesto il Leave -> Elenco (Set) di chi ha approvato
    std::map<uint32_t, std::set<uint32_t>> m_pendingLeaves;
    // Mappa: ID del drone sospettato "Morto" -> Elenco (Set) di chi vota per cacciarlo
    std::map<uint32_t, std::set<uint32_t>> m_pendingEvictions;
    // Lista dei droni che IO considero inattivi (da comunicare agli altri)
    std::set<uint32_t> m_myEvictionVotes;
    // Lista dei droni che IO so voler uscire (da comunicare agli altri, sostituisce m_recentLeaves)
    std::set<uint32_t> m_myLeaveVotes; 
};

#endif