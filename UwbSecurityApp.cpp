#include "UwbSecurityApp.h"
#include "DsTwr.h"
#include "UwbHeader.h"
#include "SimulationLogger.h"

#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/rng-seed-manager.h"

#include <cmath>
#include <queue>
#include <iomanip>

NS_LOG_COMPONENT_DEFINE ("UwbSecurityApp");
NS_OBJECT_ENSURE_REGISTERED (UwbSecurityApp);

uint32_t UwbSecurityApp::s_verbosity = 1;
bool UwbSecurityApp::s_rangingDsTwr = false;
std::shared_ptr<gnss::Constellation> UwbSecurityApp::s_constellation = nullptr;
void UwbSecurityApp::SetGnssConstellation(std::shared_ptr<gnss::Constellation> c) { s_constellation = std::move(c); }

void UwbSecurityApp::SetRangingDsTwr(bool on) { s_rangingDsTwr = on; UwbHeader::SetDsTwrMode(on); }
std::map<std::pair<char, uint32_t>, double> UwbSecurityApp::s_lastAnnounce;

void UwbSecurityApp::SetVerbosity(uint32_t level) { s_verbosity = level; }

bool UwbSecurityApp::AnnounceOnce(char kind, uint32_t droneId, double now) {
    auto key = std::make_pair(kind, droneId);
    auto it = s_lastAnnounce.find(key);
    if (it != s_lastAnnounce.end() && now - it->second < 1.0) return false;
    s_lastAnnounce[key] = now;
    return true;
}

// ---------------------------------------------------------------------------
// Soglia Mahalanobis per allarme spoofing.
// ---------------------------------------------------------------------------
static constexpr double MAHAL_OK_THRESHOLD    = 2.0;
static constexpr int    CONSECUTIVE_NEEDED    = 10;

static constexpr double c = 299792458.0;

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
    m_myLastRangeTime.assign(swarmSize, -1.0);

    uint64_t base = RngSeedManager::GetSeed() * 6364136223846793005ULL
              + RngSeedManager::GetRun();
    std::seed_seq seq{ (uint32_t)(base & 0xFFFFFFFFu),
                   (uint32_t)(base >> 32),
                   m_id };                            // tag: nodo
m_rng.seed(seq);
    std::uniform_real_distribution<double> dist_offset(-1e-9, 1e-9);
    m_clockOffset = dist_offset(m_rng);

    if (s_constellation) {
        m_gnss = std::make_unique<gnss::Receiver>(s_constellation, base ^ (0x9E3779B97F4A7C15ULL * (m_id + 1)) ^ 0x47AA55ULL);
    }

    if (s_rangingDsTwr) {
        // Il tempo "di protocollo" (slot, eta' dei range) resta allineato: la fisica dei timestamp
        // di ranging e' tutta nell'orologio UWB, con offset su tutto il contatore e skew +-20 ppm
        m_clockOffset = 0.0;
        std::seed_seq cseq{ (uint32_t)(base & 0xFFFFFFFFu), (uint32_t)(base >> 32), m_id, 0xC10CC10Cu };
        m_clockRng.seed(cseq);
        std::uniform_real_distribution<double> ppm(-20.0, 20.0);
        std::uniform_int_distribution<uint64_t> off(0, uwbclock::WRAP - 1);
        double skew = ppm(m_clockRng);
        uint64_t offset = off(m_clockRng);
        m_uwbClock = uwbclock::Clock(skew, offset, 10e-12);
    }
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

    if (m_isGuest) {
        double frameDuration = m_swarmSize * m_slotDuration;
        m_macStateEvent = Simulator::Schedule(Seconds(frameDuration), &UwbSecurityApp::EvaluateMacState, this);
    }
}

void UwbSecurityApp::StopApplication() {
    Simulator::Cancel(m_macStateEvent);
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

    // --- NUOVO FILTRO MAC STATE ---
    if (m_isGuest) {
        // Se sono fuori dal geofence o sto solo ascoltando, ho la "bocca cucita"
        if (m_macState == STATE_OUT_OF_RANGE || m_macState == STATE_LISTENING) {
            return; 
        }
    }

    if (m_slotMap.find(m_id) == m_slotMap.end() || m_slotMap[m_id] == UINT32_MAX) return;

    //if (!m_isActive || m_slotMap[m_id] == UINT32_MAX) return; 
    
    double now = Simulator::Now().GetSeconds();
    
    // --- 1. CONTROLLO INATTIVITÀ (TIMEOUT) ---
    double frameDuration = m_swarmSize * m_slotDuration;
    double warmupTime    = 3.0 * frameDuration;
    double timeoutLimit  = std::max(5.0 * frameDuration, 2.0);
    if (now > warmupTime) {
        for (auto const& [peerId, lastTime] : m_lastKnownTime) {
            if (m_pendingLeaves.count(peerId)) continue; 
            if (m_slotMap.count(peerId) && m_slotMap[peerId] != UINT32_MAX) {
                if (now - lastTime > timeoutLimit) {
                    m_pendingEvictions[peerId].insert(m_id);
                }
            }
        }
    }

    Eigen::Vector3d myGps = GetCurrentGpsPosition();
    double localTime = now + m_clockOffset;

    // 1. PRIMA dichiariamo l'oggetto header e i dati base
    UwbHeader header;
    header.SetSenderId(m_id);
    header.SetTxTimestampPs((uint64_t)(localTime * 1e12));
    header.SetGpsPosition(myGps.x(), myGps.y(), myGps.z());
    header.SetImLeaving(m_imLeaving);

    if (s_rangingDsTwr) {
        // Timestamp UWB di trasmissione + istanti in cui ho ricevuto l'ultimo pacchetto di ciascun vicino
        uint64_t txStamp = m_uwbClock.Stamp(now, m_clockRng);
        header.SetUwbTx(m_txSeq, txStamp);
        std::vector<UwbHeader::RxReport> reports;
        for (const auto& [peer, rx] : m_lastRx)
            if (now - rx.rxGlobal <= MaxRangeAge()) reports.push_back({peer, rx.seq, rx.rxStamp, rx.los});
        header.SetRxReports(reports);
        m_txHistory[m_txSeq] = { txStamp, now, myGps };
        for (auto it = m_txHistory.begin(); it != m_txHistory.end(); )
            it = (now - it->second.tGlobal > 2.0) ? m_txHistory.erase(it) : std::next(it);
        m_txSeq = (m_txSeq + 1) & 0x7F;   // 7 bit: 128 valori, la storia copre 2 s
    }
    
    // Pulizia dei timeout per gli EKF vecchi
    for (auto it = m_lastKnownTime.begin(); it != m_lastKnownTime.end(); ++it) {
        uint32_t peerId = it->first;
        if (now - it->second > 2.0) {
            m_alarms[peerId] = false;     
            m_alarmCounter[peerId] = 0;
            m_ekfBank.erase(peerId);
        }
    }

    // 2. NUOVA LOGICA: Creazione degli allarmi compressi (Bitset Dinamico)
    uint32_t numBytes = (m_swarmSize + 7) / 8; 
    std::vector<uint8_t> compressedAlarms(numBytes, 0); 

    for (const auto& [peerId, isAlarmed] : m_alarms) {
        if (isAlarmed && m_slotMap.count(peerId) && m_slotMap[peerId] != UINT32_MAX) {
            uint32_t peerSlot = m_slotMap[peerId];
            
            if (peerSlot < m_swarmSize) {
                uint32_t byteIndex = peerSlot / 8; 
                uint32_t bitIndex  = peerSlot % 8; 
                
                // Accende il bit corrispondente
                compressedAlarms[byteIndex] |= (1 << bitIndex); 
            }
        }
    }

    // 3. Assegnamo gli allarmi compressi all'header
    header.SetAlarmsList(compressedAlarms);

    // --- 2. PREPARAZIONE GOSSIP PER L'HEADER ---
    std::vector<uint32_t> leaveGossip;
    for (const auto& pair : m_pendingLeaves) {
        leaveGossip.push_back(pair.first); 
    }
    header.SetGossipLeaves(leaveGossip);

    std::vector<uint32_t> evictGossip;
    for (const auto& pair : m_pendingEvictions) {
        evictGossip.push_back(pair.first); 
    }
    header.SetGossipEvictions(evictGossip);
    
    // Condivido i range verso i MAX_RANGES_TO_SHARE vicini PIU' VICINI: la coda tiene in cima il range
    // piu' grande, che viene scartato quando si supera il limite.
    const size_t MAX_RANGES_TO_SHARE = 10;
    std::priority_queue<std::pair<double, uint32_t>> maxHeap; // (range, targetId)
    for (const auto& pair : m_slotMap) {
        uint32_t targetId = pair.first;
        if (targetId < m_myLastRanges.size()) {
            double r = m_myLastRanges[targetId];
            double age = Simulator::Now().GetSeconds() - m_myLastRangeTime[targetId];

            if (r > 0.0 && age <= MaxRangeAge()) {   // un range troppo vecchio verrebbe scartato comunque
                maxHeap.push({r, targetId});
                
                if (maxHeap.size() > MAX_RANGES_TO_SHARE) {
                    maxHeap.pop(); 
                }
            }
        }
    }

    while (!maxHeap.empty()) {
        auto bestRange = maxHeap.top();
        maxHeap.pop();
        
        double rangeAge = Simulator::Now().GetSeconds() - m_myLastRangeTime[bestRange.second];
        header.SetSharedRange(bestRange.second, bestRange.first, rangeAge);
    }


    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(header);

    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), m_port));

    if (m_pendingLeave) {
        if (s_verbosity >= 1) std::cout << ">>> GOODBYE TX: drone ID=" << m_id
                  << " ha inviato il messaggio <leave> a t="
                  << now << "s — radio off, attende consenso." << std::endl;
        //m_isActive     = false;
        this->SetActive(false);
        m_imLeaving   = false;
        m_pendingLeave = false;
    }

    // ==========================================================
    // --- DEBUG: STAMPA STATO VOTAZIONI ---
    // ==========================================================
    // Stato delle votazioni di membership: solo quando c'e' qualcosa in sospeso
    if (m_id == 0 && s_verbosity >= 2 && (!m_pendingLeaves.empty() || !m_pendingEvictions.empty())) {
        std::cout << "\n--- [DEBUG VOTAZIONI t=" << now << "s] ---" << std::endl;
        
        uint32_t activeNodes = 0;
        for (const auto& pair : m_slotMap) {
            if (pair.second != UINT32_MAX) activeNodes++; 
        }
        uint32_t quorum = (activeNodes > 2) ? ((activeNodes * 2 + 2) / 3) : activeNodes;
        
// Richiama GetSerializedSize() sull'header per sapere quanto pesa in quel momento
std::cout << "Nodi Attivi: " << activeNodes 
          << " | Quorum Richiesto: " << quorum 
          << " | Payload (Size): " << header.GetSerializedSize() << " byte" << std::endl;        
        if (m_pendingLeaves.empty()) {
            std::cout << "Leave Volontari: Nessuno in sospeso." << std::endl;
        } else {
            for (const auto& pair : m_pendingLeaves) {
                std::cout << "Leave Volontario [Drone " << pair.first << "] -> Voti (" 
                          << pair.second.size() << "/" << quorum << "): { ";
                for (uint32_t voter : pair.second) std::cout << voter << " ";
                std::cout << "}" << std::endl;
            }
        }

        if (m_pendingEvictions.empty()) {
            std::cout << "Evictions (Guasti): Nessuno in sospeso." << std::endl;
        } else {
            for (const auto& pair : m_pendingEvictions) {
                std::cout << "Eviction Guasto [Drone " << pair.first << "] -> Voti (" 
                          << pair.second.size() << "/" << quorum << "): { ";
                for (uint32_t voter : pair.second) std::cout << voter << " ";
                std::cout << "}" << std::endl;
            }
        }
        std::cout << "--------------------------------------\n" << std::endl;
    }
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

        Ptr<MobilityModel> myMob = GetNode()->GetObject<MobilityModel>();
        ns3::Vector myPos = myMob->GetPosition();

        // 1. Le coordinate assolute dell'area di missione (il cubo rosso)
        double MIN_X = 70.0;  double MAX_X = 130.0;
        double MIN_Y = 70.0;  double MAX_Y = 130.0;

        // 2. Calcolo della distanza matematica dal bordo del cubo
        double dx = std::max({0.0, MIN_X - myPos.x, myPos.x - MAX_X});
        double dy = std::max({0.0, MIN_Y - myPos.y, myPos.y - MAX_Y});
        double distanceToCube = std::sqrt(dx*dx + dy*dy);

        // 1. IL GEOFENCE: Se sento per la prima volta un nodo a meno di 10m
        if (m_isGuest && m_macState == STATE_OUT_OF_RANGE && distanceToCube/*distFisica*/ <= 10.0) {
            if (s_verbosity >= 1) std::cout << "\n>>> [GEOFENCE RF] Drone " << GetNode()->GetId()
                      << " capta segnale a " << distanceToCube/*distFisica*/ << "m. Entra in Ascolto Passivo." << std::endl;
            m_macState = STATE_LISTENING;
            m_listenCounter = 0;
        }

        // Ospite fuori dal geofence: non fa ancora parte dello sciame, non elabora il pacchetto
        if (m_isGuest && m_macState == STATE_OUT_OF_RANGE) continue;

        // 2. FASE DI ASCOLTO: l'ospite ascolta in silenzio per qualche frame prima di annunciarsi
        if (m_isGuest && m_macState == STATE_LISTENING) continue;
        // 4. AGGIUNTA DINAMICA PER I DRONI BASE (Aggiornano la loro mappa)
        if (m_slotMap.find(senderId) == m_slotMap.end() || m_slotMap[senderId] == UINT32_MAX) {
            if (s_verbosity >= 2) std::cout << ">>> [RETE] Il nodo " << m_id << " riconosce un nuovo membro attivo: Drone " << senderId << std::endl;
            m_slotMap[senderId] = senderId; // Aggiunge il nuovo arrivato alla mappa TDMA
        }
        // ==========================================================
        // --- 1. GESTIONE VOTAZIONI (GOSSIP) ---
        if (header.GetImLeaving()) {
            if (m_slotMap.count(senderId) && m_slotMap[senderId] != UINT32_MAX) {
                m_pendingLeaves[senderId].insert(senderId); 
                m_pendingLeaves[senderId].insert(m_id);     
            }
        }

        for (uint32_t peerId : header.GetGossipLeaves()) {
            if (m_slotMap.count(peerId) && m_slotMap[peerId] != UINT32_MAX) {
                m_pendingLeaves[peerId].insert(senderId);
                m_pendingLeaves[peerId].insert(m_id); 
            }
        }
        
        for (uint32_t peerId : header.GetGossipEvictions()) {
            if (m_pendingLeaves.count(peerId)) continue; 
            if (m_slotMap.count(peerId) && m_slotMap[peerId] != UINT32_MAX) {
                m_pendingEvictions[peerId].insert(senderId);
                if (m_lastKnownTime.count(peerId)) {
                    double frameDur = m_swarmSize * m_slotDuration;
                    double tLimit   = std::max(5.0 * frameDur, 2.0);
                    double now2     = Simulator::Now().GetSeconds();
                    if (now2 - m_lastKnownTime[peerId] > tLimit) {
                        m_pendingEvictions[peerId].insert(m_id);
                    }
                }
            }
        }

        // --- 2. CONTROLLO QUORUM DINAMICO ---
        uint32_t activeNodes = 0;
        for (const auto& pair : m_slotMap) {
            if (pair.second != UINT32_MAX) activeNodes++;
        }
        uint32_t quorum = (activeNodes > 2) ? ((activeNodes * 2 + 2) / 3) : activeNodes;

        // A) Evictions 
        for (auto it = m_pendingEvictions.begin(); it != m_pendingEvictions.end(); ) {
            if (m_slotMap.count(it->first) && m_slotMap[it->first] != UINT32_MAX) { // <-- FIX
                if (it->second.size() >= quorum) {
                    if (s_verbosity >= 2 || (s_verbosity >= 1 && AnnounceOnce('E', it->first, Simulator::Now().GetSeconds())))
                    std::cout << ">>> [QUORUM EVICTION] t=" << Simulator::Now().GetSeconds() 
                              << "s | Espulsione forzata Drone " << it->first << std::endl;
                    RemovePeer(it->first);
                    ReorganizeSlots(it->first);
                    it = m_pendingEvictions.erase(it); 
                    continue; 
                }
            } else {
                it = m_pendingEvictions.erase(it);
                continue;
            }
            ++it;
        }

        // B) Leave volontari
        for (auto it = m_pendingLeaves.begin(); it != m_pendingLeaves.end(); ) {
            if (m_slotMap.count(it->first) && m_slotMap[it->first] != UINT32_MAX) { // <-- FIX
                if (it->second.size() >= quorum) {
                    if (s_verbosity >= 2 || (s_verbosity >= 1 && AnnounceOnce('L', it->first, Simulator::Now().GetSeconds())))
                    std::cout << ">>> [QUORUM LEAVE] t=" << Simulator::Now().GetSeconds() 
                              << "s | Drone " << it->first << " uscito con consenso globale." << std::endl;
                    RemovePeer(it->first);
                    ReorganizeSlots(it->first);
                    it = m_pendingLeaves.erase(it); 
                    continue;
                }
            } else {
                it = m_pendingLeaves.erase(it);
                continue;
            }
            ++it;
        }

        // --- 3. LOGICA DI EKF E ALARMS ---
        std::vector<uint8_t> rxAlarms = header.GetAlarmsList();
        
        for (uint32_t j = 0; j < m_swarmSize; ++j) {
            uint32_t byteIndex = j / 8;
            uint32_t bitIndex  = j % 8;

            // Controllo di sicurezza: se il mittente ha inviato meno byte del previsto, ci fermiamo
            if (byteIndex >= rxAlarms.size()) break;

            // Leggiamo se il bit è acceso o spento
            bool vote = (rxAlarms[byteIndex] & (1 << bitIndex)) != 0;
            
            // Da qui in poi, la tua logica originale rimane IDENTICA:
            uint32_t targetId = UINT32_MAX;
            for (const auto& pair : m_slotMap) {
                if (pair.second == j && pair.second != UINT32_MAX) { 
                    targetId = pair.first; 
                    break; 
                }
            }

            if (targetId != UINT32_MAX && targetId != m_id) {
                if (vote) m_peerVotes[targetId].insert(senderId);
                else      m_peerVotes[targetId].erase(senderId);
            }
        }

        double txTimeSec = header.GetTxTimestampPs() / 1e12;
        Eigen::Vector3d claimedGps(header.GetGpsX(), header.GetGpsY(), header.GetGpsZ());

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

        for (const auto& pair : m_slotMap) {
            uint32_t i = pair.first; 
            if (pair.second == UINT32_MAX) continue;
            
            double r = header.GetSharedRange(i);
            if (r > 0.0) {
                m_networkRanges[senderId][i]     = r;
                m_networkRangeTimes[senderId][i] = txTimeSec - header.GetSharedRangeAge(i);  // istante della misura
                m_networkRangesLos[senderId][i]  = true; 
            }
        }

        ProcessRanging(senderId, claimedGps, txTimeSec, header);
    }
}

void UwbSecurityApp::ProcessRanging(uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec,
                                    const UwbHeader& header)
{
    double currentTime = Simulator::Now().GetSeconds();

    Ptr<MobilityModel> myMobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d myTruePos( myMobility->GetPosition().x, myMobility->GetPosition().y, myMobility->GetPosition().z);

    Ptr<MobilityModel> senderMobility = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    Eigen::Vector3d senderTruePos( senderMobility->GetPosition().x, senderMobility->GetPosition().y, senderMobility->GetPosition().z);

    ChannelCondition cond = m_channel->ComputeChannelCondition(senderId, m_id, senderTruePos, myTruePos, 0.0);

    Ptr<Application> app = NodeList::GetNode(senderId)->GetApplication(0);
    Ptr<UwbSecurityApp> senderApp = DynamicCast<UwbSecurityApp>(app);

    if (!senderApp) {
        NS_LOG_WARN("Applicazione UwbSecurityApp non trovata sul nodo mittente.");
        return; 
    }

    double senderOffset = senderApp->GetClockOffset();
    double trueGlobalTxTime = txTimeSec - senderOffset;
    double distTrue = (myTruePos - senderTruePos).norm();
    double tof_true = distTrue / c;
    double trueGlobalRxTime = trueGlobalTxTime + tof_true + (cond.ranging_error_m / c);
    double measuredToa = trueGlobalRxTime + m_clockOffset;

    // Misura diretta verso il mittente: valore, istante a cui si riferisce, ancora (mio GPS), LOS
    double myMeasuredRange = -1.0;
    bool   haveDirect   = false;
    double directTime   = currentTime;
    Eigen::Vector3d directAnchor(0, 0, 0);   // DS-TWR: mio GPS al messaggio centrale (vedi sotto)
    bool   directLos    = cond.is_los;

    if (!s_rangingDsTwr) {
        myMeasuredRange = (measuredToa - txTimeSec) * c;   // ToA a una via
        haveDirect = true;
    } else {
        // Il timestamp di ricezione lo produce il mio orologio UWB all'istante vero di arrivo
        uint64_t rxStamp = m_uwbClock.Stamp(trueGlobalRxTime, m_clockRng);
        RxRec cur{ header.GetSeq(), header.GetUwbTxStamp(), rxStamp, trueGlobalTxTime, trueGlobalRxTime, cond.is_los };
        auto prevIt = m_lastRx.find(senderId);
        if (prevIt != m_lastRx.end()) {
            const RxRec& prev = prevIt->second;
            // Scambio A(prev) -> io(seq s) -> A(ora): A mi dice quando ha ricevuto il mio pacchetto s
            for (const auto& rep : header.GetRxReports()) {
                if (rep.id != m_id) continue;
                auto txIt = m_txHistory.find(rep.seq);
                if (txIt == m_txHistory.end()) break;
                const TxRec& mine = txIt->second;
                if (!(prev.txGlobal < mine.tGlobal && mine.tGlobal < trueGlobalTxTime)) break;
                uwbclock::DsTwrStamps st{ prev.txStamp, rep.rxStamp, cur.txStamp,
                                          prev.rxStamp, mine.stamp, rxStamp };
                double r = uwbclock::DsTwrRange(st);
                if (r > 0.0 && r < 1000.0) {
                    myMeasuredRange = r;
                    haveDirect   = true;
                    directTime   = mine.tGlobal;   // al primo ordine e' la distanza al messaggio centrale
                    directAnchor = mine.gps;
                    directLos    = prev.los && cur.los && rep.los;   // LOS solo se tutte e tre le ricezioni lo sono
                }
                break;
            }
        }
        m_lastRx[senderId] = cur;
    }

    if (haveDirect) {
        m_myLastRanges[senderId]    = myMeasuredRange;
        m_myLastRangesLos[senderId] = directLos;
        m_myLastRangeTime[senderId] = directTime;
    }

    if (m_ekfBank.find(senderId) == m_ekfBank.end()) {
        m_ekfBank[senderId].Init(claimedGps);
        m_lastCalcTime[senderId]  = currentTime;
        m_ekfInitTime[senderId]   = currentTime;
        m_alarms[senderId]        = false;
        m_alarmCounter[senderId]  = 0;
        m_okCounter[senderId]     = 0;
        return; 
    }

    double dt = currentTime - m_lastCalcTime[senderId];
    if (dt <= 0) return;
    m_ekfBank[senderId].Predict(dt);

    std::vector<EKF::Msmnt> inputData;

    if (!s_rangingDsTwr) {
        EKF::Msmnt myData;
        myData.anchor_pos   = GetCurrentGpsPosition();   // qui, come prima: l'ordine delle estrazioni GPS conta
        myData.toa          = measuredToa;
        myData.tx_timestamp = txTimeSec;
        myData.is_direct    = true;
        myData.is_los       = cond.is_los;
        inputData.push_back(myData);
    } else if (haveDirect) {
        EKF::Msmnt myData;
        myData.anchor_pos   = directAnchor;
        myData.is_direct    = true;
        myData.clock_bias   = false;            // DS-TWR: distanza gia' priva di offset di orologio
        myData.range        = myMeasuredRange;
        myData.delay        = std::max(0.0, currentTime - directTime);
        myData.is_los       = directLos;
        inputData.push_back(myData);
    }

    double maxAge = MaxRangeAge();

    for (const auto& pair : m_slotMap) {
        uint32_t k = pair.first; 
        if (pair.second == UINT32_MAX) continue;
        
        if (k == m_id || k == senderId) continue;
        // Un drone sotto allarme collettivo non e' un'ancora affidabile: la sua posizione dichiarata
        // e' sospetta e trascinerebbe le stime dei droni onesti (avvelenamento delle ancore)
        auto alarmIt = m_collectiveAlarm.find(k);
        if (alarmIt != m_collectiveAlarm.end() && alarmIt->second) continue;

        auto rangeIt = m_networkRanges.find(k);
        if (rangeIt == m_networkRanges.end()) continue;
        auto rangeToSender = rangeIt->second.find(senderId);
        if (rangeToSender == rangeIt->second.end()) continue;
        if (rangeToSender->second <= 0.0) continue;

        double age = currentTime - m_networkRangeTimes[k][senderId];
        if (age > maxAge) continue;

        if (!m_lastKnownGps.count(k) || !m_lastKnownTime.count(k)) continue;

        // Il range e' stato misurato da k all'istante tMeas: riporto l'ancora k a quell'istante.
        // Velocita' di k: quella del mio EKF su k se c'e' (piu' stabile), altrimenti differenza di GPS.
        double tMeas = m_networkRangeTimes[k][senderId];
        Eigen::Vector3d peerGps = m_lastKnownGps[k];
        double dtAnchor = tMeas - m_lastKnownTime[k];
        if (std::fabs(dtAnchor) < 1.0) {
            auto ekfK = m_ekfBank.find(k);
            if (ekfK != m_ekfBank.end() && currentTime - m_ekfInitTime[k] > 1.0)
                peerGps += ekfK->second.GetVelocity() * dtAnchor;
            else if (m_lastKnownVelocity.count(k))
                peerGps += m_lastKnownVelocity[k] * dtAnchor;
        }

        EKF::Msmnt peerData;
        peerData.anchor_pos    = peerGps;
        peerData.delay         = std::max(0.0, currentTime - tMeas);
        peerData.is_direct     = false;
        peerData.range         = m_networkRanges[k][senderId];
        peerData.is_los        = m_networkRangesLos.count(k) ? (m_networkRangesLos[k].count(senderId) ? m_networkRangesLos[k][senderId] : true) : true;
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

    double adaptiveThreshold = std::max(2.0, 3.5 * posStd);

    // Sospetto se la posizione stimata con i range e' lontana da quella dichiarata (GPS).
    // La Mahalanobis della misura diretta non entra nella decisione: il suo residuo e' assorbito
    // dallo stato di bias di clock, quindi non vede lo spoofing e reagisce solo ai picchi NLOS
    // (resta nel CSV come diagnostica)
    bool suspiciousNow = (euclError > adaptiveThreshold);

    double timeSinceInit = currentTime - m_ekfInitTime[senderId];
    if (timeSinceInit < 2.0 * m_swarmSize * m_slotDuration * 10)
        suspiciousNow = false;

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
    uint32_t peerVoteCount = m_peerVotes.count(senderId) ? (uint32_t)m_peerVotes[senderId].size() : 0u;
    uint32_t totalVotes  = myVote + peerVoteCount;
    
    uint32_t activeObservers = 0;
    for (auto& [id, ranges] : m_networkRanges) {
        if (m_slotMap.count(id) && m_slotMap[id] != UINT32_MAX) {
            if (ranges.count(senderId) && currentTime - m_networkRangeTimes[id][senderId] < maxAge)
                activeObservers++;
        }
    }
    uint32_t threshold = std::max(2u, ((activeObservers * 2u + 2u) / 3u));
    
    bool collectiveAlarm = (totalVotes >= threshold);
    m_collectiveAlarm[senderId] = collectiveAlarm;

    // Traccia continua: osservatore 1 che stima il nodo 0
    if (s_verbosity >= 2 && m_id == 1 && senderId == 0) {
        std::cout << "t=" << currentTime
                  << " sender=" << senderId
                  << " n_peer=" << inputData.size() - 1
                  << " mahal="   << mahal
                  << " ekf_err=" << (estimatedPos - senderTruePos).norm()
                  << " claimed_err=" << (claimedGps - senderTruePos).norm()
                  << " alarm="   << m_alarms[senderId]
                  << " myVote=" << myVote
                  << " peerVotes=" << peerVoteCount
                  << " totalVotes=" << totalVotes
                  << " threshold=" << threshold
                  << " collective=" << collectiveAlarm
                  << std::endl;
    }

    if (m_csv && m_csv->is_open()) {
        Eigen::Vector3d recoveredPos = collectiveAlarm ? estimatedPos : claimedGps;

        uint32_t currentActiveNodes = 0;
        for (const auto& pair : m_slotMap) {
            if (pair.second != UINT32_MAX) currentActiveNodes++;
        }

        SimulationLogger::LogObservation(
            currentTime, senderId, m_id,
            estimatedPos, claimedGps, senderTruePos,
            collectiveAlarm, recoveredPos, *m_csv, totalVotes, threshold,
            currentActiveNodes, peerVoteCount,
            myVote, suspiciousNow, mahal, posStd);

        // Stampa solo quando cambia il voto individuale o l'allarme collettivo di questo osservatore
        uint32_t voteState = myVote * 2u + (collectiveAlarm ? 1u : 0u);
        auto prev = m_lastPrintedVoteState.find(senderId);
        bool changed = (prev == m_lastPrintedVoteState.end()) ? (voteState != 0u) : (prev->second != voteState);
        m_lastPrintedVoteState[senderId] = voteState;
        if (s_verbosity >= 2 && changed) {
            std::cout << "[DEBUG t=" << currentTime << "s] " 
                      << "Osservatore ID=" << m_id 
                      << " | Nodi attivi visti: " << currentActiveNodes 
                      << " | Voti ricevuti dai Peer: " << peerVoteCount 
                      << " | Mio Voto: " << myVote
                      << " | TOTALE: " << totalVotes << " (Soglia: " << threshold << ")" 
                      << std::endl;
        }
    }
}

Eigen::Vector3d UwbSecurityApp::GetCurrentGpsPosition() {
    Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d gps(mobility->GetPosition().x,
                        mobility->GetPosition().y,
                        mobility->GetPosition().z);

    if (m_gnss) {
        // Ricevitore GNSS simulato: errore comune (uguale per i droni vicini) + errore individuale
        gnss::Fix f = m_gnss->Measure(gps, Simulator::Now().GetSeconds());
        if (f.ok) { gps = f.pos; m_lastGnssFix = f; }
    } else {
        std::normal_distribution<double> noise_xy(0.0, 0.2);
        std::normal_distribution<double> noise_z(0.0, 0.4);
        gps.x() += noise_xy(m_rng);
        gps.y() += noise_xy(m_rng);
        gps.z() += noise_z(m_rng);
    }

    if (m_isMalicious) {
        const double TARGET_OFFSET = 15.0; 
        const double RAMP_DURATION = 10.0; 
        double elapsed  = Simulator::Now().GetSeconds() - m_attackStartTime;
        double progress = std::min(1.0, std::max(0.0, elapsed / RAMP_DURATION));
        gps.y() += TARGET_OFFSET * progress;
    }
    return gps;
}

void UwbSecurityApp::SetActive(bool active) {
    m_isActive = active;
    
    if (!active) {
        m_lastKnownGps.clear();
        m_lastKnownTime.clear();
        m_lastKnownVelocity.clear();
        m_ekfBank.clear();
        m_alarms.clear();
        m_alarmCounter.clear();
        m_okCounter.clear();
        m_networkRanges.clear();
        m_networkRangeTimes.clear();
        m_networkRangesLos.clear();
        m_peerVotes.clear();
        m_slotMap.clear();
        
        m_pendingLeaves.clear();
        m_pendingEvictions.clear();
        m_myEvictionVotes.clear();
        m_myLeaveVotes.clear();
        
        m_myLastRanges.assign(m_swarmSize, -1.0);
        m_myLastRangesLos.assign(m_swarmSize, true);
        m_myLastRangeTime.assign(m_swarmSize, -1.0);
    }
}

void UwbSecurityApp::ScheduleLeave() {
    if (m_pendingLeave) return; 
    m_pendingLeave = true;
    m_imLeaving    = true;
    if (s_verbosity >= 1) std::cout << ">>> LEAVE SCHEDULATO: drone ID=" << m_id
              << " invierà goodbye al prossimo slot TDMA (t="
              << Simulator::Now().GetSeconds() << "s)" << std::endl;
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
    m_collectiveAlarm.erase(peerId);
    m_lastRx.erase(peerId);
    if (peerId < m_myLastRanges.size()) {   // non condivido piu' il mio range verso chi e' uscito
        m_myLastRanges[peerId]    = -1.0;
        m_myLastRangeTime[peerId] = -1.0;
    }

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
    //m_swarmSize = (uint32_t)m_slotMap.size();
}

void UwbSecurityApp::ReorganizeSlots(uint32_t leavingDroneId) {
    if (m_slotMap.find(leavingDroneId) == m_slotMap.end()) {
        return; 
    }

    m_slotMap[leavingDroneId] = UINT32_MAX;
    
    m_lastKnownGps.erase(leavingDroneId);
    m_ekfBank.erase(leavingDroneId);

    if (m_id == 0 && s_verbosity >= 2) {
        std::cout << "\n[TDMA STATIC FRAME] Il Drone " << leavingDroneId 
                  << " è uscito. Il suo slot è disattivato." << std::endl;
    }
}

void UwbSecurityApp::SetNodeRole(bool isGuest) {
    m_isGuest = isGuest;
    if (isGuest) {
        m_macState = STATE_OUT_OF_RANGE;
        m_listenCounter = 0;
        m_isActive = true;
    } else {
        m_macState = STATE_ACTIVE;
        m_isActive = true;
    }
}

void UwbSecurityApp::EvaluateMacState() {
    if (!m_isGuest) return;
    if (!m_isActive) return;   // dopo il goodbye l'ospite e' fuori: la macchina a stati si ferma

    double frameDuration = m_swarmSize * m_slotDuration;

    // =================================================================
    // --- NUOVO: CONTROLLO ISTERESI (USCITA VOLONTARIA A 14 METRI) ---
    // =================================================================
    if (m_macState != STATE_OUT_OF_RANGE) {
        Ptr<MobilityModel> myMob = GetNode()->GetObject<MobilityModel>();
        ns3::Vector myPos = myMob->GetPosition();

        double MIN_X = 70.0;  double MAX_X = 130.0;
        double MIN_Y = 70.0;  double MAX_Y = 130.0;

        double dx = std::max({0.0, MIN_X - myPos.x, myPos.x - MAX_X});
        double dy = std::max({0.0, MIN_Y - myPos.y, myPos.y - MAX_Y});
        double distanceToCube = std::sqrt(dx*dx + dy*dy);

        // Se mi sono allontanato oltre 14 metri (Isteresi: 10 per entrare, 14 per uscire)
        if (distanceToCube > 14.0) {
            if (m_macState == STATE_ACTIVE && !m_pendingLeave) {
                if (s_verbosity >= 1) std::cout << "\n>>> [ISTERESI GEOFENCE] Drone " << m_id
                          << " ha superato i 14m dal cubo (" << distanceToCube 
                          << "m). Inizio procedura di uscita volontaria (Graceful Leave)." << std::endl;
                ScheduleLeave(); 
            } else if (m_macState == STATE_LISTENING || m_macState == STATE_JOINING) {
                if (s_verbosity >= 1) std::cout << "\n>>> [ISTERESI] Drone " << m_id
                          << " si è allontanato durante il Join. Abortito." << std::endl;
                m_macState = STATE_OUT_OF_RANGE;
                m_listenCounter = 0;
            }
        }
    }


    if (m_macState == STATE_LISTENING) {
        m_listenCounter++;
        if (m_listenCounter >= 3) {
            // TDMA statico: ogni possibile membro ha uno slot riservato, uguale al suo ID
            m_macState = STATE_JOINING;
            m_slotId = m_id;
            m_slotMap[m_id] = m_id;
            if (s_verbosity >= 1)
                std::cout << "\n>>> [NODO GUEST] Drone " << m_id << ": ascolto completato, mi annuncio nel mio slot (" << m_id << ")" << std::endl;
        }
    } else if (m_macState == STATE_JOINING) {
        m_macState = STATE_ACTIVE;
        if (s_verbosity >= 1) std::cout << ">>> [NODO " << m_id << "] Join completato, ora sono ACTIVE." << std::endl;
    }

    // Ri-schedula il timer per il prossimo frame
    m_macStateEvent = Simulator::Schedule(Seconds(frameDuration), &UwbSecurityApp::EvaluateMacState, this);
}
