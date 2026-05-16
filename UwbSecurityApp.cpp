#include "UwbSecurityApp.h"
#include "UwbHeader.h"
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "SimulationLogger.h"
#include <cmath>

NS_LOG_COMPONENT_DEFINE ("UwbSecurityApp");
NS_OBJECT_ENSURE_REGISTERED (UwbSecurityApp);

// ---------------------------------------------------------------------------
// Soglia Mahalanobis per allarme spoofing.
//
// La distanza di Mahalanobis segue (approssimativamente) una distribuzione
// chi con n gradi di libertà. Per n=4 misure, chi(4, 99.9%) ≈ 4.64.
// Usiamo una soglia conservativa per ridurre i falsi positivi.
// ---------------------------------------------------------------------------
// Soglia Mahalanobis calcolata SOLO sulla misura diretta (1 grado di libertà).
// Con 1 misura: Mahal = |y_direct| / sqrt(S_direct)
// In assenza di attacco vale circa 1.0 (rumore normale).
// Soglia = 5.0 → errore ranging > 5 sigma (probabilità < 0.00006%)
static constexpr double MAHAL_ALARM_THRESHOLD = 5.0;
static constexpr double MAHAL_OK_THRESHOLD    = 3.0;
// Campioni consecutivi necessari per alzare/abbassare l'allarme.
// Con slot=5ms e 10 droni, 1 giro = 50ms → 10 campioni = 500ms di persistenza
static constexpr int    CONSECUTIVE_NEEDED    = 10;

static constexpr double c = 299792458.0; // [m/s]

TypeId UwbSecurityApp::GetTypeId (void) {
    static TypeId tid = TypeId ("UwbSecurityApp")
        .SetParent<Application>()
        .SetGroupName("Custom")
        .AddConstructor<UwbSecurityApp>();
    return tid;
}

UwbSecurityApp::UwbSecurityApp()
    : m_id(0), m_swarmSize(6), m_isMalicious(false),
      m_attackStartTime(0.0), m_slotDuration(0.0),
      m_port(9), m_csv(nullptr) {}

UwbSecurityApp::~UwbSecurityApp() { m_socket = 0; }

void UwbSecurityApp::Setup(uint32_t id, uint32_t swarmSize, double slotDuration,
                            Ptr<UWBChannel> channel, std::ofstream* csv) {
    m_id          = id;
    m_slotId      = id;
    m_swarmSize   = swarmSize;
    m_channel     = channel;
    m_csv         = csv;
    m_slotDuration = slotDuration;
    m_myLastRanges.assign(swarmSize, -1.0);
    m_myLastRangesLos.assign(swarmSize, true);

    m_rng.seed(m_id + 12345);
    std::uniform_real_distribution<double> dist_offset(-1e-9, 1e-9);
    m_clockOffset = dist_offset(m_rng);
}

void UwbSecurityApp::SetMalicious(bool isMalicious) {
    if (isMalicious && !m_isMalicious)
        m_attackStartTime = Simulator::Now().GetSeconds();
    m_isMalicious = isMalicious;
}
bool UwbSecurityApp::IsMalicious() const { return m_isMalicious; }

void UwbSecurityApp::StartApplication() {
    if (!m_socket) {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
    }
    m_socket->SetRecvCallback(MakeCallback(&UwbSecurityApp::ReceivePacket, this));
    m_socket->SetAllowBroadcast(true);

    double firstTxTime = m_id * m_slotDuration;
    m_sendEvent = Simulator::Schedule(Seconds(firstTxTime),
                                       &UwbSecurityApp::SendUwbMessage, this);
}

void UwbSecurityApp::StopApplication() {
    if (m_socket) m_socket->Close();
    Simulator::Cancel(m_sendEvent);
}

// ---------------------------------------------------------------------------
// TX: broadcast del proprio stato + range condivisi
// ---------------------------------------------------------------------------

void UwbSecurityApp::SendUwbMessage() {
    m_sendEvent = Simulator::Schedule(Seconds(m_swarmSize * m_slotDuration),
                                       &UwbSecurityApp::SendUwbMessage, this);

    if (!m_isActive) return;
    
    Eigen::Vector3d myGps = GetCurrentGpsPosition();
    double currentSimTime = Simulator::Now().GetSeconds();
    double localTime = currentSimTime + m_clockOffset;

    UwbHeader header;
    header.SetSenderId(m_id);
    header.SetTxTimestampPs((uint64_t)(localTime * 1e12));
    header.SetGpsPosition(myGps.x(), myGps.y(), myGps.z());
    header.SetVoteBitmask(GetVoteBitmask());
    header.SetImLeaving(m_imLeaving);

    for (const auto& pair : m_slotMap) {
        uint32_t targetId = pair.first;
        if (targetId < m_myLastRanges.size()) {
            header.SetSharedRange(targetId, m_myLastRanges[targetId]);
        }
    }

    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(header);
    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), m_port));

    // Se avevamo schedulato un leave, questo era il messaggio di goodbye:
    // ora ci disattiviamo definitivamente.
    if (m_pendingLeave) {
        std::cout << ">>> GOODBYE TX: drone ID=" << m_id
                  << " ha inviato il messaggio di leave a t="
                  << currentSimTime << "s — radio off." << std::endl;
        m_isActive    = false;
        m_imLeaving   = false;
        m_pendingLeave = false;
    }
    //Eigen::Vector3d myGps = GetCurrentGpsPosition();
    //m_sendEvent = Simulator::Schedule(Seconds(m_swarmSize * m_slotDuration),
    //                                   &UwbSecurityApp::SendUwbMessage, this);
}

void UwbSecurityApp::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;

    if (!m_isActive) {
        while ((packet = socket->RecvFrom(from))) {}
        return;
    }
    while ((packet = socket->RecvFrom(from))) {
        UwbHeader header;
        packet->RemoveHeader(header);

        uint32_t senderId = header.GetSenderId();
        if (senderId == m_id) continue;

        // --- Gestione messaggio di LEAVE ---
        if (header.GetImLeaving()) {
            std::cout << ">>> GOODBYE RX: drone ID=" << m_id
                      << " ha ricevuto il messaggio di leave da drone ID=" << senderId
                      << " a t=" << Simulator::Now().GetSeconds() << "s" << std::endl;
            RemovePeer(senderId);
            // --- Gestione messaggio di LEAVE ---
        if (header.GetImLeaving()) {
            std::cout << ">>> GOODBYE RX: drone ID=" << m_id
                      << " ha ricevuto il messaggio di leave da drone ID=" << senderId
                      << " a t=" << Simulator::Now().GetSeconds() << "s" << std::endl;
            RemovePeer(senderId);
            ReorganizeSlots(senderId); // <--- AGGIUNGI QUESTA RIGA
            continue;
        }
            continue;
        }
        
        uint32_t receivedMask = header.GetVoteBitmask();
        for (const auto& pair : m_slotMap) {
            uint32_t targetId = pair.first;
            bool senderSuspectsTarget = !((receivedMask >> targetId) & 1u); 
            if (senderSuspectsTarget) {
                m_peerVotes[targetId].insert(senderId);  
            } else {
                m_peerVotes[targetId].erase(senderId);    
            }
        }  

        double txTimeSec = header.GetTxTimestampPs() / 1e12;
        Eigen::Vector3d claimedGps(header.GetGpsX(), header.GetGpsY(), header.GetGpsZ());

        //cancella
        if (m_lastKnownGps.count(senderId) && m_lastKnownTime.count(senderId)) {
           double dt_gps = txTimeSec - m_lastKnownTime[senderId];
            if (dt_gps > 0.001 && dt_gps < 2.0) {
                Eigen::Vector3d vel = (claimedGps - m_lastKnownGps[senderId]) / dt_gps;
                if (vel.norm() < 50.0) { 
                    m_lastKnownVelocity[senderId] = vel;
                }
            }
        }

        m_lastKnownGps[senderId]  = claimedGps;
        m_lastKnownTime[senderId] = txTimeSec;

        for (uint32_t i = 0; i < m_swarmSize; ++i) {
            double r = header.GetSharedRange(i);
            if (r > 0.0) {
                m_networkRanges[senderId][i]     = r;
                m_networkRangeTimes[senderId][i] = txTimeSec;
                m_networkRangesLos[senderId][i]  = true; // conservativo
            }
        }

        ProcessRanging(senderId, claimedGps, txTimeSec);
    }
}

void UwbSecurityApp::ProcessRanging(uint32_t senderId,
                                      Eigen::Vector3d claimedGps,
                                      double txTimeSec)
{
    double currentTime = Simulator::Now().GetSeconds();

    Ptr<MobilityModel> myMobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d myTruePos( myMobility->GetPosition().x, myMobility->GetPosition().y, myMobility->GetPosition().z);

    Ptr<MobilityModel> senderMobility = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    Eigen::Vector3d senderTruePos( senderMobility->GetPosition().x, senderMobility->GetPosition().y, senderMobility->GetPosition().z);

    ChannelCondition cond = m_channel->ComputeChannelCondition(senderTruePos, myTruePos, 0.0);

    Ptr<Application> app = NodeList::GetNode(senderId)->GetApplication(0);
    Ptr<UwbSecurityApp> senderApp = DynamicCast<UwbSecurityApp>(app);
    double senderOffset = senderApp->GetClockOffset();
    
    double trueGlobalTxTime = txTimeSec - senderOffset;
    
    double distTrue = (myTruePos - senderTruePos).norm();
    double tof_true = distTrue / c;
    
    double trueGlobalRxTime = trueGlobalTxTime + tof_true + (cond.ranging_error_m / c);
    
    double measuredToa = trueGlobalRxTime + m_clockOffset;

    double myMeasuredRange = (measuredToa - txTimeSec) * c;
    m_myLastRanges[senderId]    = myMeasuredRange;
    m_myLastRangesLos[senderId] = cond.is_los;

    if (m_ekfBank.find(senderId) == m_ekfBank.end()) {
        m_ekfBank[senderId].Init(claimedGps);
        m_lastCalcTime[senderId]  = currentTime;
        m_alarms[senderId]        = false;
        m_alarmCounter[senderId]  = 0;
        m_okCounter[senderId]     = 0;
        return; 
    }

    double dt = currentTime - m_lastCalcTime[senderId];
    if (dt <= 0) return;
    m_ekfBank[senderId].Predict(dt);

    std::vector<EKF::Msmnt> inputData;

    EKF::Msmnt myData;
    myData.anchor_pos   = GetCurrentGpsPosition();
    myData.toa          = measuredToa;
    myData.tx_timestamp = txTimeSec;
    myData.is_direct    = true;
    myData.is_los       = cond.is_los;
    inputData.push_back(myData);

    double maxAge = std::min(2.0, 2.0 * m_swarmSize * m_slotDuration);

    for (uint32_t k = 0; k < m_swarmSize; ++k) {
        if (k == m_id || k == senderId) continue;

        auto rangeIt = m_networkRanges.find(k);
        if (rangeIt == m_networkRanges.end()) continue;
        auto rangeToSender = rangeIt->second.find(senderId);
        if (rangeToSender == rangeIt->second.end()) continue;
        if (rangeToSender->second <= 0.0) continue;

        double age = currentTime - m_networkRangeTimes[k][senderId];
        if (age > maxAge) continue;

        if (!m_lastKnownGps.count(k) || !m_lastKnownTime.count(k)) continue;

        Eigen::Vector3d peerGps = m_lastKnownGps[k];
        double peerAge = currentTime - m_lastKnownTime[k];
        if (m_lastKnownVelocity.count(k) && peerAge < 1.0)
        peerGps += m_lastKnownVelocity[k] * peerAge;

        EKF::Msmnt peerData;
        peerData.anchor_pos    = peerGps;
        peerData.is_direct     = false;
        peerData.range         = m_networkRanges[k][senderId];
        peerData.is_los        = m_networkRangesLos.count(k) ?
                         (m_networkRangesLos[k].count(senderId) ?
                          m_networkRangesLos[k][senderId] : true) : true;
        inputData.push_back(peerData);
    }

    if ((int)inputData.size() >= 4) {
        m_ekfBank[senderId].Update(inputData);
    }
    m_lastCalcTime[senderId] = currentTime;


    double mahal  = m_ekfBank[senderId].GetMahalanobisDistance();
    double posStd = m_ekfBank[senderId].GetPositionStdDev();
    Eigen::Vector3d estimatedPos = m_ekfBank[senderId].GetPosition();
    double euclError = (estimatedPos - claimedGps).norm();

    double adaptiveThreshold = std::max(6.0, 3.5 * posStd);

    bool suspiciousNow = (mahal  > MAHAL_ALARM_THRESHOLD) ||
                         (euclError > adaptiveThreshold);

    if (currentTime < 15.0) suspiciousNow = false;

    if (suspiciousNow) {
        m_alarmCounter[senderId]++;
        m_okCounter[senderId] = 0;
    } else {
        m_okCounter[senderId]++;
        m_alarmCounter[senderId] = 0;
    }

    if (m_alarmCounter[senderId] >= CONSECUTIVE_NEEDED)
        m_alarms[senderId] = true;
    if (m_okCounter[senderId] >= CONSECUTIVE_NEEDED)
        m_alarms[senderId] = false;

    uint32_t myVote = m_alarms[senderId] ? 1u : 0u;
    uint32_t peerVoteCount = m_peerVotes.count(senderId)
                             ? (uint32_t)m_peerVotes[senderId].size()
                             : 0u;
    uint32_t totalVotes  = myVote + peerVoteCount;
    uint32_t activeNodes = (uint32_t)m_lastKnownGps.size() + 1;
    uint32_t threshold   = std::max(2u, (activeNodes * 2u) / 3u);

    bool collectiveAlarm = (totalVotes >= threshold);

    if (m_id == 1 && senderId == 0) {
        std::cout << "t=" << currentTime
                  << " sender=" << senderId
                  << " n_peer=" << inputData.size() - 1
                  << " mahal="   << mahal
                  << " ekf_err=" << (estimatedPos - senderTruePos).norm()
                  << " claimed_err=" << (claimedGps - senderTruePos).norm()
                  << " alarm="   << m_alarms[senderId]
                  << std::endl;
    }

    if (m_csv && m_csv->is_open()) {
        Eigen::Vector3d recoveredPos = collectiveAlarm ? estimatedPos : claimedGps;

        SimulationLogger::LogObservation(
            currentTime, senderId, m_id,
            estimatedPos, claimedGps, senderTruePos,
            collectiveAlarm, recoveredPos, *m_csv);
    }
}

Eigen::Vector3d UwbSecurityApp::GetCurrentGpsPosition() {
    Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d gps(mobility->GetPosition().x,
                        mobility->GetPosition().y,
                        mobility->GetPosition().z);

    std::normal_distribution<double> noise_xy(0.0, 0.2); 
    std::normal_distribution<double> noise_z(0.0, 0.4);

    gps.x() += noise_xy(m_rng);
    gps.y() += noise_xy(m_rng);
    gps.z() += noise_z(m_rng);

    if (m_isMalicious) {
        const double TARGET_OFFSET = 15.0; 
        const double RAMP_DURATION = 10.0; 
        double elapsed  = Simulator::Now().GetSeconds() - m_attackStartTime;
        double progress = std::min(1.0, std::max(0.0, elapsed / RAMP_DURATION));
        gps.y() += TARGET_OFFSET * progress;
    }
    return gps;
}

uint32_t UwbSecurityApp::GetVoteBitmask() {
    uint32_t mask = 0xFFFFFFFF;
    for (auto const& pair : m_alarms)
        if (pair.second) mask &= ~(1u << pair.first);
    return mask;
}

void UwbSecurityApp::SetActive(bool active) {
    m_isActive = active;
}

// ---------------------------------------------------------------------------
// ScheduleLeave — il drone aspetta il suo prossimo slot TDMA naturale,
// invia un messaggio con m_imLeaving=true, poi si disattiva da solo.
// ---------------------------------------------------------------------------
void UwbSecurityApp::ScheduleLeave() {
    if (m_pendingLeave) return;   // chiamata doppia, ignora
    m_pendingLeave = true;
    m_imLeaving    = true;
    std::cout << ">>> LEAVE SCHEDULATO: drone ID=" << m_id
              << " invierà goodbye al prossimo slot TDMA (t="
              << Simulator::Now().GetSeconds() << "s)" << std::endl;
}

void UwbSecurityApp::AddPeer(uint32_t peerId) {
    m_lastKnownGps.erase(peerId);
    m_lastKnownTime.erase(peerId);
    m_lastKnownVelocity.erase(peerId);
    m_ekfBank.erase(peerId);
    m_alarms[peerId] = false;
    m_alarmCounter[peerId] = 0;
    m_okCounter[peerId] = 0;
}

void UwbSecurityApp::RemovePeer(uint32_t peerId) {
    m_lastKnownGps.erase(peerId);
    m_lastKnownTime.erase(peerId);
    m_lastKnownVelocity.erase(peerId);
    m_ekfBank.erase(peerId);
    m_alarms.erase(peerId);
    m_networkRanges.erase(peerId);
    m_networkRangeTimes.erase(peerId);
    m_networkRangesLos.erase(peerId);

    m_peerVotes.erase(peerId);
    for (auto& [id, voters] : m_peerVotes)
        voters.erase(peerId);
}

void UwbSecurityApp::InitSlotMap(const std::vector<uint32_t>& activeIds) {
    m_slotMap.clear();
    uint32_t slot = 0;
    for (uint32_t droneId : activeIds) {
        m_slotMap[droneId] = slot++;
    }
    if (m_slotMap.count(m_id))
        m_slotId = m_slotMap[m_id];
    m_swarmSize = (uint32_t)m_slotMap.size();
}

void UwbSecurityApp::AddPeerSlot(uint32_t peerId, uint32_t slotId) {
    m_slotMap[peerId] = slotId;
    m_swarmSize = (uint32_t)m_slotMap.size();
}

void UwbSecurityApp::ReorganizeSlots(uint32_t leavingDroneId) {
    if (m_slotMap.find(leavingDroneId) == m_slotMap.end()) {
        return; 
    }

    uint32_t emptySlot = m_slotMap[leavingDroneId];
    
    uint32_t maxSlot = 0;
    uint32_t lastDroneId = leavingDroneId; 
    
    for (const auto& pair : m_slotMap) {
        if (pair.second > maxSlot) {
            maxSlot = pair.second;
            lastDroneId = pair.first;
        }
    }

    m_slotMap.erase(leavingDroneId);

    if (lastDroneId != leavingDroneId) {
        m_slotMap[lastDroneId] = emptySlot;
    }

    if (m_slotMap.count(m_id)) {
        m_slotId = m_slotMap[m_id];
    }
    
    m_swarmSize = (uint32_t)m_slotMap.size();
}