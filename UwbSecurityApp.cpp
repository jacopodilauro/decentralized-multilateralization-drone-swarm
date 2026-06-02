#include "UwbSecurityApp.h"
#include "UwbHeader.h"
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "SimulationLogger.h"

#include "ns3/netsimulyzer-module.h"

#include <cmath>
#include <queue>

NS_LOG_COMPONENT_DEFINE ("UwbSecurityApp");
NS_OBJECT_ENSURE_REGISTERED (UwbSecurityApp);

// ---------------------------------------------------------------------------
// Soglia Mahalanobis per allarme spoofing.
// ---------------------------------------------------------------------------
//static constexpr double MAHAL_ALARM_THRESHOLD = 3.5;
static constexpr double MAHAL_ALARM_THRESHOLD = 5;
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

    /*m_sendEvent = Simulator::Schedule(Seconds(m_swarmSize * m_slotDuration),
                                       &UwbSecurityApp::SendUwbMessage, this);
    */
    // NUOVO CODICE (Sincronizzazione del Tempo basata sul Consenso EKF)

    double frameDuration = m_swarmSize * m_slotDuration;
    double averageBias = 0.0;
    int validPeers = 0;

    // 1. Calcoliamo la media dello sfasamento (Clock Bias) rispetto allo sciame
    for (auto& pair : m_ekfBank) {
        uint32_t peerId = pair.first;
        
        // Sicurezza: Sincronizziamoci SOLO con i droni di cui ci fidiamo (non in allarme)
        if (!m_alarms[peerId]) { 
            averageBias += pair.second.GetClockBias();
            validPeers++;
        }
    }

    if (validPeers > 0) {
        averageBias /= validPeers;
    }

    // 2. Applichiamo un Controllore Proporzionale (Smorzamento)
    // Non correggiamo tutto l'errore in un colpo solo per evitare "rimbalzi" e instabilità.
    // Correggiamo solo il 10% dell'errore (Kp = 0.1) ad ogni ciclo.
    double Kp = 0.1; 
    double correction = averageBias * Kp;

    // 3. Sicurezza: Limitiamo la correzione massima (es. max 5% della durata di uno slot)
    // Questo impedisce a un errore improvviso dell'EKF di sballare completamente il TDMA
    double maxCorrection = m_slotDuration * 0.05;
    correction = std::max(-maxCorrection, std::min(maxCorrection, correction));

    // 4. Scheduliamo il prossimo invio con il tempo corretto!
    double nextTxTime = frameDuration + correction;
    m_sendEvent = Simulator::Schedule(Seconds(nextTxTime), &UwbSecurityApp::SendUwbMessage, this);

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
    
    // tolgo temporaneamente la condivisione dei range per semplificare il debug, ma l'idea è che ogni drone condivida i range misurati verso gli altri peer attivi, così da avere più osservatori per il controllo incrociato e aumentare la resilienza contro i falsi positivi/negativi.
    /*for (const auto& pair : m_slotMap) {
        uint32_t targetId = pair.first;
        if (targetId < m_myLastRanges.size()) {
            header.SetSharedRange(targetId, m_myLastRanges[targetId]);
        }
    }*/
    const size_t MAX_RANGES_TO_SHARE = 10;
    std::priority_queue<std::pair<double, uint32_t>> maxHeap; // (range, targetId)
    for (const auto& pair : m_slotMap) {
        uint32_t targetId = pair.first;
        if (targetId < m_myLastRanges.size()) {
            double r = m_myLastRanges[targetId];
            
            if (r > 0.0) { 
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
        
        header.SetSharedRange(bestRange.second, bestRange.first);
    }


    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(header);

    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), m_port));
    // --- INIZIO EFFETTO VISIVO TRASMISSIONE ---
    Ptr<netsimulyzer::NodeConfiguration> nodeConfig = GetNode()->GetObject<netsimulyzer::NodeConfiguration>();
    if (nodeConfig) {
        // 1. Cambia il colore del drone (es. Verde fluo) mentre trasmette
        nodeConfig->SetAttribute("BaseColor", netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(255u, 165u, 0u)));
        
        // 2. Genera l'onda radio (opzionale, ma consigliato insieme al cambio colore)
        //nodeConfig->Transmit(Seconds(m_slotDuration), 100.0, netsimulyzer::Color3(0, 255, 0));

        // 3. Schedula il ritorno al colore originale alla fine del suo slot
        Simulator::Schedule(Seconds(m_slotDuration), &UwbSecurityApp::RevertColor, this);
    }
    // --- FINE EFFETTO VISIVO TRASMISSIONE ---

    if (m_pendingLeave) {
        std::cout << ">>> GOODBYE TX: drone ID=" << m_id
                  << " ha inviato il messaggio <leave> a t="
                  << now << "s — radio off, attende consenso." << std::endl;
        //m_isActive     = false;
        this->SetActive(false);
        m_imLeaving   = false;
        m_pendingLeave = false;
        m_macState     = STATE_OUT_OF_RANGE;  // aggiungi
        m_localSlotMap.clear();               // aggiungi
        m_localOccupiedSlots.clear(); 
    }

    // ==========================================================
    // --- DEBUG: STAMPA STATO VOTAZIONI ---
    // ==========================================================
    if (m_id == 0 && ((now >= 79.0 && now <= 85.0) || (now >= 99.0 && now <= 105.0) || (now >= 149.0 && now <= 155.0))) {
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

    bool canListen = m_isGuest && (m_macState == STATE_LISTENING);

    if (!m_isActive && !canListen) {
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
            std::cout << "\n>>> [GEOFENCE RF] Drone " << GetNode()->GetId() 
                      << " capta segnale a " << distanceToCube/*distFisica*/ << "m. Entra in Ascolto Passivo." << std::endl;
            m_macState = STATE_LISTENING;
            m_listenCounter = 0;
            m_localSlotMap.clear();
            m_localOccupiedSlots.clear();
        }

        // 2. FASE DI ASCOLTO: Mappatura silenziosa
        if (m_isGuest && m_macState == STATE_LISTENING) {
            // Mappo l'ID di chi sta parlando
                m_localSlotMap[senderId] = true; 
                if (m_slotMap.count(senderId) && m_slotMap[senderId] != UINT32_MAX) {
                    m_localOccupiedSlots.insert(m_slotMap[senderId]);
                }else {
        // Fallback: per i base drone id == slot, quindi va bene come proxy
                m_localOccupiedSlots.insert(senderId);
                }
            continue;
        }
        // Trova lo slot usato dal mittente guardando la mappa
        uint32_t senderSlot = UINT32_MAX;
        if (m_slotMap.find(senderId) != m_slotMap.end()) {
            senderSlot = m_slotMap[senderId];
        }

        // 3. FASE DI CONTESA: Rilevamento collisioni
        if (m_isGuest && m_macState == STATE_JOINING) {
            if (m_slotMap.count(senderId) && m_slotMap[senderId] == (uint32_t)m_chosenSlot) {
                std::cout << "[COLLISIONE!] Qualcun altro sta usando l'ID " << senderSlot << ". Backoff applicato.\n";
                m_macState = STATE_LISTENING; // Ritorno in ascolto, ho perso lo slot
                m_listenCounter = 0;
                continue;
            }
        }

        // 4. AGGIUNTA DINAMICA PER I DRONI BASE (Aggiornano la loro mappa)
        if (m_slotMap.find(senderId) == m_slotMap.end() || m_slotMap[senderId] == UINT32_MAX) {
            std::cout << ">>> [RETE] Il nodo " << m_id << " riconosce un nuovo membro attivo: Drone " << senderId << std::endl;
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
                    std::cout << "\n=======================================================\n"
                              << " [WARNING] ESPULSIONE ESEGUITA! \n"
                              << " Drone Sospetto ID : " << it->first << "\n"
                              << " Tempo Simulazione : " << Simulator::Now().GetSeconds() << "s\n"
                              << " Quorum Raggiunto  : " << it->second.size() << " voti su " << quorum << " richiesti.\n"
                              << "=======================================================\n" << std::endl;
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
                m_networkRangeTimes[senderId][i] = txTimeSec;
                m_networkRangesLos[senderId][i]  = true; 
            }
        }

        ProcessRanging(senderId, claimedGps, txTimeSec);
    }
}

void UwbSecurityApp::ProcessRanging(uint32_t senderId, Eigen::Vector3d claimedGps, double txTimeSec)
{
    double currentTime = Simulator::Now().GetSeconds();

    Ptr<MobilityModel> myMobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d myTruePos( myMobility->GetPosition().x, myMobility->GetPosition().y, myMobility->GetPosition().z);

    Ptr<MobilityModel> senderMobility = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    Eigen::Vector3d senderTruePos( senderMobility->GetPosition().x, senderMobility->GetPosition().y, senderMobility->GetPosition().z);

    ChannelCondition cond = m_channel->ComputeChannelCondition(senderTruePos, myTruePos, 0.0);

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
    double myMeasuredRange = (measuredToa - txTimeSec) * c;
    m_myLastRanges[senderId]    = myMeasuredRange;
    m_myLastRangesLos[senderId] = cond.is_los;

    /*if (m_ekfBank.find(senderId) == m_ekfBank.end()) {
        m_ekfBank[senderId].Init(claimedGps);
        m_lastCalcTime[senderId]  = currentTime;
        m_ekfInitTime[senderId]   = currentTime;
        m_alarms[senderId]        = false;
        m_alarmCounter[senderId]  = 0;
        m_okCounter[senderId]     = 0;
        return; 
    }*/
   if (m_ekfBank.find(senderId) == m_ekfBank.end()) {
    // Prova a stimare la posizione iniziale dai range noti
    Eigen::Vector3d initPos = claimedGps;
    int validPeers = 0;
    
    for (const auto& [peerId, ranges] : m_networkRanges) {
        if (ranges.count(senderId) && ranges.at(senderId) > 0.0 
            && m_lastKnownGps.count(peerId)) {
            initPos += m_lastKnownGps[peerId];
            validPeers++;
        }
    }
    if (validPeers > 0) initPos /= (validPeers + 1); // media con claimedGps
    
    m_ekfBank[senderId].Init(initPos);
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

    EKF::Msmnt myData;
    myData.anchor_pos   = GetCurrentGpsPosition();
    myData.toa          = measuredToa;
    myData.tx_timestamp = txTimeSec;
    myData.is_direct    = true;
    myData.is_los       = cond.is_los;
    inputData.push_back(myData);

    double maxAge = std::min(1.0, 8.0 * m_swarmSize * m_slotDuration);

    for (const auto& pair : m_slotMap) {
        uint32_t k = pair.first; 
        if (pair.second == UINT32_MAX) continue;
        
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

    double adaptiveThreshold = std::max(5.0, 4 * posStd);

    bool suspiciousNow = (mahal  > MAHAL_ALARM_THRESHOLD) ||
                         (euclError > adaptiveThreshold);

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
    //uint32_t threshold = /*std::max(2u, */((activeObservers * 2u + 2u) / 3u)/*)*/;
    uint32_t threshold = (activeObservers > 2) ? ((activeObservers * 2u + 2u) / 3u) : activeObservers;
    bool collectiveAlarm = (totalVotes >= threshold);

    if (m_id == 1 && senderId == 0) {
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
        
        
        double clockBias = m_ekfBank[senderId].GetClockBias();

        SimulationLogger::LogObservation(
            currentTime, senderId, m_id,
            estimatedPos, claimedGps, senderTruePos,
            collectiveAlarm, recoveredPos, *m_csv, totalVotes, threshold,
            /*currentActiveNodes*/activeObservers, peerVoteCount, clockBias); 

            // --- INIZIO DEBUGGER ESTREMO ---
        if (currentTime >= 238.0 && currentTime <= 239.0 && senderId == 0 && m_id == 1) {
            std::cout << "\n[DEBUGGER C++] STO SCRIVENDO NEL CSV A t=" << currentTime << "s"
                      << "\n -> Nodi fisici scritti (activeObservers): " << activeObservers
                      << "\n -> Soglia scritta (threshold): " << threshold 
                      << "\n------------------------------------------------\n" << std::endl;
        }
        // --- FINE DEBUGGER ESTREMO ---

        if (currentTime >= 200.0 && currentTime <= 210.0 && senderId == 0) {
            std::cout << "[DEBUG t=" << currentTime << "s] " 
                      << "Osservatore ID=" << m_id 
                      << " | Nodi attivi visti: " << activeObservers 
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
    uint64_t mask = 0xFFFFFFFFFFFFFFFF;
    for (auto const& pair : m_alarms)
        if (pair.second) mask &= ~(1ULL << pair.first);
    return mask;
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
        m_recentLeaves.clear(); 
        
        m_pendingLeaves.clear();
        m_pendingEvictions.clear();
        m_myEvictionVotes.clear();
        m_myLeaveVotes.clear();
        
        m_myLastRanges.assign(m_swarmSize, -1.0);
        m_myLastRangesLos.assign(m_swarmSize, true);
    }
}

void UwbSecurityApp::ScheduleLeave() {
    if (m_pendingLeave) return; 
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
    //m_swarmSize = (uint32_t)m_slotMap.size();
}

void UwbSecurityApp::AddPeerSlot(uint32_t peerId, uint32_t slotId) {
    m_slotMap[peerId] = slotId;
    //m_swarmSize = (uint32_t)m_slotMap.size();
}

void UwbSecurityApp::ReorganizeSlots(uint32_t leavingDroneId) {
    if (m_slotMap.find(leavingDroneId) == m_slotMap.end()) {
        return; 
    }

    m_slotMap[leavingDroneId] = UINT32_MAX;
    
    m_lastKnownGps.erase(leavingDroneId);
    m_ekfBank.erase(leavingDroneId);

    if (m_id == 0) {
        std::cout << "\n[TDMA STATIC FRAME] Il Drone " << leavingDroneId 
                  << " è uscito. Il suo slot è disattivato." << std::endl;
    }
}

void UwbSecurityApp::PrintTerminalDashboard() {
    double currentTime = Simulator::Now().GetSeconds();
    
    if (m_id != 1) return;

    std::ofstream os("dashboard_log.txt", std::ios::app);

    std::string header = "\n=========================================================================================\n"
                         "   UWB NETWORK SECURITY & TDMA STATUS DASHBOARD | Time: " + std::to_string(currentTime) + "s\n"
                         "-----------------------------------------------------------------------------------------\n";
    
    std::cout << header;
    if (os.is_open()) os << header;

    std::cout << std::left << std::setw(8) << "Node ID" << std::setw(10) << "TDMA Slot"
              << std::setw(12) << "Mahalanobis" << std::setw(14) << "EKF Error(m)"
              << std::setw(14) << "GPS Error(m)" << std::setw(14) << "Clock Bias(s)"
              << std::setw(10) << "Status" << std::setw(10) << "Quorum\n";
    std::cout << std::string(89, '-') << "\n";
    
    if (os.is_open()) {
        os << std::left << std::setw(8) << "Node ID" << std::setw(10) << "TDMA Slot"
           << std::setw(12) << "Mahalanobis" << std::setw(14) << "EKF Error(m)"
           << std::setw(14) << "GPS Error(m)" << std::setw(14) << "Clock Bias(s)"
           << std::setw(10) << "Status" << std::setw(10) << "Quorum\n";
        os << std::string(89, '-') << "\n";
    }

    for (const auto& pair : m_slotMap) {
        uint32_t peerId = pair.first;
        uint32_t slotId = pair.second;

        // Se lo slot è vuoto, non stamparlo a schermo
        if (slotId == UINT32_MAX) continue;

        if (peerId == m_id) continue; 

        double mahal = 0.0;
        double ekfErr = 0.0;
        double gpsErr = 0.0;
        double clockBias = 0.0;
        std::string status = "OK";

        if (m_ekfBank.find(peerId) != m_ekfBank.end()) {
            EKF& peerEkf = m_ekfBank[peerId];
            mahal = peerEkf.GetMahalanobisDistance();
            clockBias = peerEkf.GetClockBias(); 
            
            Ptr<MobilityModel> peerMob = NodeList::GetNode(peerId)->GetObject<MobilityModel>();
            if (peerMob) {
                Eigen::Vector3d truePos(peerMob->GetPosition().x, peerMob->GetPosition().y, peerMob->GetPosition().z);
                ekfErr = (peerEkf.GetPosition() - truePos).norm();
                if (m_lastKnownGps.count(peerId)) {
                    gpsErr = (m_lastKnownGps[peerId] - truePos).norm();
                }
            }
        }

        if (m_alarms.count(peerId) && m_alarms[peerId]) status = "SPOOFED";

        uint32_t positiveVotes = (status == "SPOOFED") ? 1 : 0;
        uint32_t totalVoters = 0;

        for(const auto& p : m_slotMap) { if(p.second != UINT32_MAX) totalVoters++; }
        
        if (m_peerVotes.count(peerId)) positiveVotes += m_peerVotes[peerId].size();
        
        std::string quorumStr = std::to_string(positiveVotes) + "/" + std::to_string(totalVoters);

        std::cout << std::left << std::fixed << std::setprecision(3)
                  << std::setw(8) << peerId << std::setw(10) << slotId
                  << std::setw(12) << mahal << std::setw(14) << ekfErr
                  << std::setw(14) << gpsErr 
                  << std::setw(14) << std::scientific << std::setprecision(2) << clockBias << std::fixed << std::setprecision(3)
                  << std::setw(10) << status << std::setw(10) << quorumStr << "\n";
                  
        if (os.is_open()) {
            os << std::left << std::fixed << std::setprecision(3)
               << std::setw(8) << peerId << std::setw(10) << slotId
               << std::setw(12) << mahal << std::setw(14) << ekfErr
               << std::setw(14) << gpsErr 
               << std::setw(14) << std::scientific << std::setprecision(2) << clockBias << std::fixed << std::setprecision(3)
               << std::setw(10) << status << std::setw(10) << quorumStr << "\n";
        }
    }
    std::cout << std::string(89, '=') << "\n" << std::endl;
    if (os.is_open()) {
        os << std::string(89, '=') << "\n\n";
        os.close();
    }
}
    uint32_t UwbSecurityApp::GetFirstAvailableSlot() {
    // Creiamo un array temporaneo per mappare quali slot sono occupati
    std::vector<bool> slotUsed(m_swarmSize, false);
    
    for (const auto& pair : m_slotMap) {
        if (pair.second != UINT32_MAX && pair.second < m_swarmSize) {
            slotUsed[pair.second] = true;
        }
    }
    
    // Cerchiamo il primo "buco" (slot = false)
    for (uint32_t i = 0; i < m_swarmSize; ++i) {
        if (!slotUsed[i]) return i; 
    }
    
    return UINT32_MAX; 
    // NOTA: In questo scenario, se il frame è pieno, il drone guest rimarrà in ascolto finché non si libera uno slot.
    // Da implementare eventualmente una logica di backoff o di attesa casuale per evitare che più droni guest si accavallino cercando di occupare lo stesso slot appena libero.
}


void UwbSecurityApp::SetNodeRole(bool isGuest) {
    m_isGuest = isGuest;
    if (isGuest) {
        m_macState = STATE_OUT_OF_RANGE;
        m_listenCounter = 0;
        m_chosenSlot = -1;
        m_isActive = true;
    } else {
        m_macState = STATE_ACTIVE;
        m_chosenSlot = m_id;
        m_isActive = true;
    }
}

void UwbSecurityApp::EvaluateMacState() {
    if (!m_isGuest) return;

    double frameDuration = m_swarmSize * m_slotDuration;

    // =================================================================
    // --- CONTROLLO ISTERESI E RIENTRO NEL GEOFENCE ---
    // =================================================================
    Ptr<MobilityModel> myMob = GetNode()->GetObject<MobilityModel>();
    ns3::Vector myPos = myMob->GetPosition();

    double MIN_X = 70.0;  double MAX_X = 130.0;
    double MIN_Y = 70.0;  double MAX_Y = 130.0;

    double dx = std::max({0.0, MIN_X - myPos.x, myPos.x - MAX_X});
    double dy = std::max({0.0, MIN_Y - myPos.y, myPos.y - MAX_Y});
    double distanceToCube = std::sqrt(dx*dx + dy*dy);

    if (m_macState != STATE_OUT_OF_RANGE) {
        // ISTERESI DI USCITA: Se mi sono allontanato oltre 14 metri
        if (distanceToCube > 14.0) {
            if (m_macState == STATE_ACTIVE && !m_pendingLeave) {
                std::cout << "\n>>> [ISTERESI GEOFENCE] Drone " << m_id
                          << " ha superato i 14m. Inizio Graceful Leave." << std::endl;
                ScheduleLeave();
            } else if (m_macState == STATE_LISTENING || m_macState == STATE_JOINING) {
                std::cout << "\n>>> [ISTERESI] Drone " << m_id
                          << " si è allontanato durante il Join. Abortito." << std::endl;
                m_macState    = STATE_OUT_OF_RANGE;
                m_listenCounter = 0;
                m_localSlotMap.clear();
                m_localOccupiedSlots.clear();
            }
        }
    } else {
        // --- BUG 1 FIX: ISTERESI DI RIENTRO ---
        // EvaluateMacState è l'unico timer che gira anche quando m_isActive=false,
        // quindi è l'unico posto sicuro dove fare il wake-up.
        // ReceivePacket non può farlo perché scarta tutto se m_isActive=false.
        if (distanceToCube <= 10.0) {
            std::cout << "\n>>> [GEOFENCE WAKE-UP] Drone " << m_id
                      << " rientrato nel cubo (dist=" << distanceToCube
                      << "m). Riaccendo la radio." << std::endl;
            m_isActive      = true;
            m_macState      = STATE_LISTENING;
            m_listenCounter = 0;
            m_localSlotMap.clear();
            m_localOccupiedSlots.clear(); // BUG 4 FIX: pulizia set slot occupati
        }
    }

    // =================================================================
    // --- LOGICA DI JOINING ---
    // =================================================================
    if (m_macState == STATE_LISTENING) {
        m_listenCounter++;
        if (m_listenCounter >= 3) {

            // BUG 4 FIX: cerchiamo uno slot libero in m_localOccupiedSlots
            // (che contiene slot reali), non in m_localSlotMap (che contiene ID).
            m_chosenSlot = -1;
            for (uint32_t i = 0; i < m_swarmSize; ++i) {
                if (m_localOccupiedSlots.find(i) == m_localOccupiedSlots.end()) {
                    m_chosenSlot = i;
                    break;
                }
            }

            if (m_chosenSlot != -1) {
                m_macState       = STATE_JOINING;
                m_slotId         = m_chosenSlot;
                m_slotMap[m_id]  = m_chosenSlot;
                std::cout << "\n>>> [NODO GUEST " << m_id << "] Slot " << m_chosenSlot
                          << " libero trovato. Tento il join." << std::endl;
            } else {
                std::cout << "[NODO GUEST " << m_id
                          << "] Frame TDMA saturo! Rimango in ascolto...\n";
                m_listenCounter = 0;
            }
        }
    } else if (m_macState == STATE_JOINING) {
        // BUG 2 FIX: ricostruiamo la slotMap dai peer visti durante il listening.
        // SetActive(false) aveva fatto m_slotMap.clear(), quindi senza questo
        // il drone riparte con solo se stesso nella mappa e non vota mai.
        for (const auto& [peerId, seen] : m_localSlotMap) {
            if (seen && m_slotMap.find(peerId) == m_slotMap.end()) {
                // I droni base usano id == slot; per i guest usiamo
                // lo slot registrato in m_localOccupiedSlots se disponibile.
                // Come proxy sicuro usiamo peerId (identico ai base drone).
                m_slotMap[peerId] = peerId;
            }
        }

        m_macState = STATE_ACTIVE;
        std::cout << ">>> [NODO " << m_id << "] Join confermato! "
                  << "Peer noti: " << m_slotMap.size()
                  << ". Ora sono ACTIVE e voto." << std::endl;
    }

    // Ri-schedula il timer per il prossimo frame
    m_macStateEvent = Simulator::Schedule(
        Seconds(frameDuration), &UwbSecurityApp::EvaluateMacState, this);
}

void UwbSecurityApp::RevertColor() {
    Ptr<netsimulyzer::NodeConfiguration> nodeConfig = GetNode()->GetObject<netsimulyzer::NodeConfiguration>();
    if (nodeConfig) {
        // Ripristina il colore in base al ruolo del drone
        if (m_id == 0) {
            // Drone Master: Rosso
            nodeConfig->SetAttribute("BaseColor", netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(255u, 0u, 0u)));
        } else if (!m_isGuest) {
            // Droni Core (Base): Blu
            nodeConfig->SetAttribute("BaseColor", netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(0u, 0u, 255u)));
        } else {
            // Droni Guest: Giallo
            nodeConfig->SetAttribute("BaseColor", netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(255u, 255u, 0u)));
        }
    }
}