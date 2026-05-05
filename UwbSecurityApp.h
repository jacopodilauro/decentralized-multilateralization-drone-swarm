#ifndef UWB_SECURITY_APP_H
#define UWB_SECURITY_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/mobility-model.h"
#include "ns3/event-id.h"
#include "UWBChannel.h"
#include "EKF.h"
#include "random"

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <fstream>

using namespace ns3;

class UwbSecurityApp : public Application {
public:
    static TypeId GetTypeId(void);

    UwbSecurityApp();
    virtual ~UwbSecurityApp();

    void Setup(uint32_t id, uint32_t swarmSize, double slotDuration,
               Ptr<UWBChannel> channel, std::ofstream* csv);

    void SetMalicious(bool isMalicious);
    bool IsMalicious() const;
    
    double GetClockOffset() const { return m_clockOffset; }

private:
    virtual void StartApplication(void) override;
    virtual void StopApplication(void)  override;

    double m_clockOffset;
    std::mt19937 m_rng;

    void SendUwbMessage();
    void ReceivePacket(Ptr<Socket> socket);
    void ProcessRanging(uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec);

    Eigen::Vector3d GetCurrentGpsPosition();
    uint32_t        GetVoteBitmask();

    uint32_t        m_id;
    uint32_t        m_swarmSize;
    bool            m_isMalicious;
    double          m_attackStartTime;
    double          m_slotDuration;
    uint16_t        m_port;
    Ptr<UWBChannel> m_channel;
    std::ofstream*  m_csv;

    Ptr<Socket> m_socket;
    EventId     m_sendEvent;

    std::vector<double> m_myLastRanges;
    std::vector<bool>   m_myLastRangesLos;

    std::map<uint32_t, Eigen::Vector3d> m_lastKnownGps;
    std::map<uint32_t, double>          m_lastKnownTime;
    std::map<uint32_t, Eigen::Vector3d> m_lastKnownVelocity;

    std::map<uint32_t, std::map<uint32_t, double>> m_networkRanges;
    std::map<uint32_t, std::map<uint32_t, double>> m_networkRangeTimes;
    std::map<uint32_t, std::map<uint32_t, bool>>   m_networkRangesLos;

    std::map<uint32_t, EKF>    m_ekfBank;
    std::map<uint32_t, double> m_lastCalcTime;

    std::map<uint32_t, bool> m_alarms;
    std::map<uint32_t, int>  m_alarmCounter;
    std::map<uint32_t, int>  m_okCounter;
};

#endif