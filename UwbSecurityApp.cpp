#include "UwbSecurityApp.h"
#include "UwbHeader.h"
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h" 
#include "SimulationLogger.h"

NS_LOG_COMPONENT_DEFINE ("UwbSecurityApp");
NS_OBJECT_ENSURE_REGISTERED (UwbSecurityApp);

TypeId UwbSecurityApp::GetTypeId (void) {
    static TypeId tid = TypeId ("UwbSecurityApp").SetParent<Application> ().SetGroupName("Custom").AddConstructor<UwbSecurityApp> ();
    return tid;
}

UwbSecurityApp::UwbSecurityApp() : m_id(0), m_swarmSize(6), m_isMalicious(false), m_attackStartTime(0.0), m_slotDuration(0.0), m_port(9), m_csv(nullptr) {
  //  for(int i=0; i<6; i++) m_myLastRanges[i] = -1.0; // Inizializza array
}
UwbSecurityApp::~UwbSecurityApp() { m_socket = 0; }

void UwbSecurityApp::Setup(uint32_t id, uint32_t swarmSize, double slotDuration, Ptr<UWBChannel> channel, std::ofstream* csv) {
    m_id = id; m_swarmSize = swarmSize; m_channel = channel; m_csv = csv;
    m_slotDuration = slotDuration;
    m_myLastRanges.assign(swarmSize, -1.0);
}

void UwbSecurityApp::SetMalicious(bool isMalicious) {
    if (isMalicious && !m_isMalicious) m_attackStartTime = Simulator::Now().GetSeconds();
    m_isMalicious = isMalicious;
}
bool UwbSecurityApp::IsMalicious() const { return m_isMalicious; }

void UwbSecurityApp::StartApplication (void) {
    if (!m_socket) {
        m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
        InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), m_port);
        m_socket->Bind (local);
    }
    m_socket->SetRecvCallback (MakeCallback (&UwbSecurityApp::ReceivePacket, this));
    m_socket->SetAllowBroadcast (true);

    double firstTxTime = m_id * m_slotDuration;
    m_sendEvent = Simulator::Schedule (Seconds (firstTxTime), &UwbSecurityApp::SendUwbMessage, this);
}

void UwbSecurityApp::StopApplication (void) {
    if (m_socket) m_socket->Close ();
    Simulator::Cancel (m_sendEvent);
}

void UwbSecurityApp::SendUwbMessage () {
    Eigen::Vector3d myGps = GetCurrentGpsPosition();
    uint32_t myVoteMask = GetVoteBitmask();
    double currentSimTime = Simulator::Now().GetSeconds();

    UwbHeader header;
    header.SetSenderId(m_id);
    header.SetTxTimestampPs((uint64_t)(currentSimTime * 1e12)); 
    header.SetGpsPosition(myGps.x(), myGps.y(), myGps.z());
    header.SetVoteBitmask(myVoteMask);

    // Trasmettiamo al mondo anche le nostre letture di distanza recenti!
    for(uint32_t i=0; i< m_swarmSize; i++) {
        header.SetSharedRange(i, m_myLastRanges[i]);
    }

    Ptr<Packet> packet = Create<Packet> ();
    packet->AddHeader(header);
    
    InetSocketAddress dest (Ipv4Address("255.255.255.255"), m_port);
    m_socket->SendTo(packet, 0, dest);

    double timeToNextTurn = m_swarmSize * m_slotDuration;
    m_sendEvent = Simulator::Schedule (Seconds (timeToNextTurn), &UwbSecurityApp::SendUwbMessage, this);
}

void UwbSecurityApp::ReceivePacket (Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {
        UwbHeader header;
        packet->RemoveHeader (header);

        uint32_t senderId = header.GetSenderId();
        if (senderId == m_id) continue; 

        double txTimeSec = header.GetTxTimestampPs() / 1e12;
        Eigen::Vector3d claimedGps(header.GetGpsX(), header.GetGpsY(), header.GetGpsZ());
        
        //salvataggio dati ricevuti in cache
        m_lastKnownGps[senderId] = claimedGps;
        m_lastKnownTime[senderId] = txTimeSec;
        for(uint32_t i=0; i < m_swarmSize; i++) {
            double r = header.GetSharedRange(i);
            
            if(r > 0.0)
            {
                m_networkRanges[senderId][i] = header.GetSharedRange(i);
                m_networkRangeTimes[senderId][i] = txTimeSec;
            }
        }
        
        ProcessRanging(senderId, claimedGps, txTimeSec);
    }
}

void UwbSecurityApp::ProcessRanging (uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec)
{
    double currentTime = Simulator::Now().GetSeconds();

    // Il canale simulato interviene qui solo per calcolare la fisica del volo del pacchetto
    Ptr<MobilityModel> myMobility = GetNode()->GetObject<MobilityModel>();
    Ptr<MobilityModel> senderMobility = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    Eigen::Vector3d myTruePos(myMobility->GetPosition().x, myMobility->GetPosition().y, myMobility->GetPosition().z);
    Eigen::Vector3d senderTruePos(senderMobility->GetPosition().x, senderMobility->GetPosition().y, senderMobility->GetPosition().z);

    ChannelCondition cond = m_channel->ComputeChannelCondition(senderTruePos, myTruePos, 0.0);
    const double c = 299792458.0;
    double distTrue = (myTruePos - senderTruePos).norm();
    double tof = distTrue / c;
    double measuredToa = txTimeSec + tof + (cond.ranging_error_m / c);

    // Salvo la MIA misurazione in locale per condividerla al mio prossimo turno
    double myMeasuredDistance = (measuredToa - txTimeSec) * c;
    m_myLastRanges[senderId] = myMeasuredDistance;

    if (m_ekfBank.find(senderId) == m_ekfBank.end()) {
        m_ekfBank[senderId].Init(claimedGps);
        m_lastCalcTime[senderId] = currentTime;
        m_alarms[senderId] = false;
        m_alarmCounter[senderId] = 0;
        m_okCounter[senderId] = 0;
    }

    double dt = currentTime - m_lastCalcTime[senderId];
    if (dt > 0) {
        m_ekfBank[senderId].Predict(dt);
        
        std::vector<EKF::Msmnt> inputData;
        
        // 1. Aggiungo la MIA misurazione locale (l'unica cosa che conosco direttamente)
        EKF::Msmnt myData;
        myData.anchor_pos = GetCurrentGpsPosition(); 
        myData.toa = measuredToa;
        myData.tx_timestamp = txTimeSec; 

        inputData.push_back(myData);

        // 2. FUSIONE DATI: Uso le misurazioni condivise via rete dagli altri droni!
        for (uint32_t k = 0; k < m_swarmSize; ++k) {
            if (k == m_id || k == senderId) continue; 
            
            // Se in passato il drone k mi ha inviato la sua distanza dal senderId
            if (m_networkRanges[k].count(senderId) && m_networkRanges[k][senderId] > 0) {
                // E se mi ricordo il suo ultimo GPS dichiarato
                if (m_lastKnownGps.count(k) && m_lastKnownTime.count(k)) {
                    
                    // Non voglio usare i dati vecchi quindi
                    double maxAge = std::min(2.0, 1.0 * m_swarmSize * m_slotDuration);
                    double age = currentTime - m_networkRangeTimes[k][senderId];
                    if (age > maxAge) continue;

                    EKF::Msmnt peerData;
                    peerData.anchor_pos = m_lastKnownGps[k]; // Mi fido del suo GPS
                    peerData.tx_timestamp = m_lastKnownTime[k]; // Adattamento per l'EKF
                    peerData.toa = m_lastKnownTime[k] + (m_networkRanges[k][senderId] / c); // Uso la sua distanza calcolata
                    
                    inputData.push_back(peerData);
                }
            }
        }

        // Il filtro scatterà SOLO se abbiamo raccolto almeno 3 informazioni, 
        // emulando un vero delay di propagazione della conoscenza!
        if (inputData.size() >= 4) {
            m_ekfBank[senderId].Update(inputData);
        }
        m_lastCalcTime[senderId] = currentTime;
    }

    Eigen::Vector3d calculatedPos = m_ekfBank[senderId].GetPosition();
    
    
    // --- DEBUG TEMPORANEO ---
    if (m_id == 1 && senderId == 0) {
        std::cout << "t=" << currentTime 
                  << " ekf_err=" << (calculatedPos - senderTruePos).norm()
                  << " claimed_err=" << (claimedGps - senderTruePos).norm()
                  << std::endl;
    }
    // --- FINE DEBUG ---


    double error = (calculatedPos - claimedGps).norm();
    
    // Logica di scatto allarme
    /* Logica vecchia, davedere
    if (error > 10.0) m_alarms[senderId] = true;
    else m_alarms[senderId] = false;
    */
    // --- inizio Logica nuova, per evitare che gli allarmi ballino
    if(error > 10) 
    {
        m_alarmCounter[senderId]++; 
        m_okCounter[senderId] = 0;
    }else
    {
        m_okCounter[senderId]++;
        m_alarmCounter[senderId] = 0;
    }
    if (m_alarmCounter[senderId] >= 5) { m_alarms[senderId] = true; }
    if (m_okCounter[senderId] >= 5) { m_alarms[senderId] = false; }
    // --- fine


    if (m_csv && m_csv->is_open()) {
        Eigen::Vector3d recoveredPos = claimedGps; 
        if (m_alarms[senderId]) recoveredPos = calculatedPos; 

        SimulationLogger::LogObservation(
            currentTime, senderId, m_id,
            calculatedPos, claimedGps, senderTruePos, 
            m_alarms[senderId], recoveredPos, *m_csv
        );
    }
}

Eigen::Vector3d UwbSecurityApp::GetCurrentGpsPosition() {
    Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d gps(mobility->GetPosition().x, mobility->GetPosition().y, mobility->GetPosition().z);

    if (m_isMalicious) {
        const double TARGET_OFFSET = 15.0; 
        const double RAMP_DURATION = 10.0; 
        double time_elapsed = Simulator::Now().GetSeconds() - m_attackStartTime;
        double progress = time_elapsed / RAMP_DURATION;
        if (progress < 0.0) progress = 0.0;
        if (progress > 1.0) progress = 1.0;
        
        gps.y() += (TARGET_OFFSET * progress); 
    }
    return gps;
}

uint32_t UwbSecurityApp::GetVoteBitmask() {
    uint32_t mask = 0xFFFFFFFF;
    for (auto const& pair : m_alarms) {
        if (pair.second) mask &= ~(1 << pair.first);
    }
    return mask;
}
