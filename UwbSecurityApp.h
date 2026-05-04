#ifndef UWB_SECURITY_APP_H
#define UWB_SECURITY_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/ptr.h"
#include "ns3/mobility-model.h"
#include <map>
#include <vector>
#include "EKF.h"
#include "UWBChannel.h"

using namespace ns3;

class UwbSecurityApp : public Application 
{
public:
    static TypeId GetTypeId (void);
    UwbSecurityApp();
    virtual ~UwbSecurityApp();
    void Setup(uint32_t id, uint32_t swarmSize, double slotDuration, Ptr<UWBChannel> channel, std::ofstream* csv);
    void SetMalicious(bool isMalicious);
    bool IsMalicious() const;

protected:
    virtual void StartApplication (void) override;
    virtual void StopApplication (void) override;

private:
    void SendUwbMessage ();
    void ReceivePacket (Ptr<Socket> socket);
    void ProcessRanging (uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec);
    Eigen::Vector3d GetCurrentGpsPosition (); 
    uint32_t GetVoteBitmask ();

    uint32_t m_id;
    uint32_t m_swarmSize;
    bool m_isMalicious;
    double m_attackStartTime;
    double m_slotDuration;

    Ptr<Socket> m_socket;
    uint16_t m_port;
    Ptr<UWBChannel> m_channel;

    std::map<int, EKF> m_ekfBank;
    std::map<int, double> m_lastCalcTime;
    std::map<int, bool> m_alarms;
    std::map<uint32_t, int> m_alarmCounter;
    std::map<uint32_t, int> m_okCounter;
    
    std::vector<double> m_myLastRanges; // Le MIE misurazioni verso gli altri
    std::map<int, Eigen::Vector3d> m_lastKnownGps; // Ultimo GPS dichiarato noto degli altri
    std::map<uint32_t, std::map<uint32_t, double>> m_networkRanges; // Le distanze misurate e dichiarate dagli ALTRI
    std::map<uint32_t, double> m_lastKnownTime;    
    std::map<uint32_t, std::map<uint32_t, double>> m_networkRangeTimes;
    EventId m_sendEvent;
    std::ofstream* m_csv;
};

#endif
