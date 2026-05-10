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

// ---------------------------------------------------------------------------
// Application lifecycle
// ---------------------------------------------------------------------------

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
    header.SetTxTimestampPs((uint64_t)(localTime * 1e12)); //currentSimTime altrimetni 
    header.SetGpsPosition(myGps.x(), myGps.y(), myGps.z());
    header.SetVoteBitmask(GetVoteBitmask());

    for (uint32_t i = 0; i < m_swarmSize; ++i)
        header.SetSharedRange(i, m_myLastRanges[i]);

    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(header);
    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), m_port));
    //Eigen::Vector3d myGps = GetCurrentGpsPosition();
    //m_sendEvent = Simulator::Schedule(Seconds(m_swarmSize * m_slotDuration),
    //                                   &UwbSecurityApp::SendUwbMessage, this);
}

// ---------------------------------------------------------------------------
// RX: parsing del pacchetto e cache dei dati
// ---------------------------------------------------------------------------

void UwbSecurityApp::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;

    if (!m_isActive) {
        // Se inattivo, svuota il buffer di rete e ignora i pacchetti
        while ((packet = socket->RecvFrom(from))) {}
        return;
    }
    // Se arrivi qui, il nodo è attivo. Inizia il tuo codice originale:
    while ((packet = socket->RecvFrom(from))) {
        UwbHeader header;
        packet->RemoveHeader(header);

        uint32_t senderId = header.GetSenderId();
        if (senderId == m_id) continue;

        double txTimeSec = header.GetTxTimestampPs() / 1e12;
        Eigen::Vector3d claimedGps(header.GetGpsX(), header.GetGpsY(), header.GetGpsZ());

        //cancella
        if (m_lastKnownGps.count(senderId) && m_lastKnownTime.count(senderId)) {
           double dt_gps = txTimeSec - m_lastKnownTime[senderId];
            if (dt_gps > 0.001 && dt_gps < 2.0) {
                Eigen::Vector3d vel = (claimedGps - m_lastKnownGps[senderId]) / dt_gps;
                if (vel.norm() < 50.0) {  // scarta valori impossibili (>50 m/s)
                    m_lastKnownVelocity[senderId] = vel;
                }
            }
        }



        // Cache GPS e timestamp del sender
        m_lastKnownGps[senderId]  = claimedGps;
        m_lastKnownTime[senderId] = txTimeSec;

        // Cache dei range condivisi dal sender verso tutti gli altri nodi
        // Nota: m_networkRanges[sender][target] = range misurato da sender verso target
        for (uint32_t i = 0; i < m_swarmSize; ++i) {
            double r = header.GetSharedRange(i);
            if (r > 0.0) {
                m_networkRanges[senderId][i]     = r;
                m_networkRangeTimes[senderId][i] = txTimeSec;
                // Condividiamo anche la LOS flag — non disponibile nel pacchetto,
                // usiamo la nostra stima locale come approssimazione
                m_networkRangesLos[senderId][i]  = true; // conservativo
            }
        }

        ProcessRanging(senderId, claimedGps, txTimeSec);
    }
}

// ---------------------------------------------------------------------------
// Core: ranging fisico + costruzione misure + EKF + allarme
// ---------------------------------------------------------------------------

void UwbSecurityApp::ProcessRanging(uint32_t senderId,
                                      Eigen::Vector3d claimedGps,
                                      double txTimeSec)
{
    double currentTime = Simulator::Now().GetSeconds();

    // -----------------------------------------------------------------------
    // 1. RANGING FISICO (solo da dati locali — nessun accesso a ground truth)
    //
    // La posizione VERA del sender è inaccessibile in un sistema reale.
    // Usiamo la nostra posizione reale (legittima: è il nostro stesso nodo)
    // e il canale UWB per simulare ciò che fisicamente misuriamo.
    //
    // In ns-3 dobbiamo ancora usare NodeList per ottenere la posizione
    // del sender per calcolare il canale — questo è l'unico punto in cui
    // lo usiamo, e rappresenta la fisica del segnale radio, NON
    // un'informazione disponibile a livello applicativo.
    // -----------------------------------------------------------------------

    Ptr<MobilityModel> myMobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d myTruePos( myMobility->GetPosition().x, myMobility->GetPosition().y, myMobility->GetPosition().z);

    // Posizione vera del sender — usata SOLO per simulare il canale fisico
    Ptr<MobilityModel> senderMobility = NodeList::GetNode(senderId)->GetObject<MobilityModel>();
    Eigen::Vector3d senderTruePos( senderMobility->GetPosition().x, senderMobility->GetPosition().y, senderMobility->GetPosition().z);

    // Simulazione fisica del canale UWB
    ChannelCondition cond = m_channel->ComputeChannelCondition(senderTruePos, myTruePos, 0.0);

    // Recuperiamo il vero offset di clock del sender per dedurre il "tempo globale"
    // (Questo trucco è solo per simulare la fisica in ns-3, il drone non "sa" questo dato)
    Ptr<Application> app = NodeList::GetNode(senderId)->GetApplication(0);
    Ptr<UwbSecurityApp> senderApp = DynamicCast<UwbSecurityApp>(app);
    double senderOffset = senderApp->GetClockOffset();
    
    // Il sender ha stampato txTimeSec (che è Locale = Globale + senderOffset)
    // Il vero tempo globale di partenza era:
    double trueGlobalTxTime = txTimeSec - senderOffset;
    
    double distTrue = (myTruePos - senderTruePos).norm();
    double tof_true = distTrue / c;
    
    // Il pacchetto arriva in questo istante reale:
    double trueGlobalRxTime = trueGlobalTxTime + tof_true + (cond.ranging_error_m / c);
    
    // MA il ricevitore legge il tempo con il SUO orologio locale:
    double measuredToa = trueGlobalRxTime + m_clockOffset;

    // Range misurato da me verso il sender
    double myMeasuredRange = (measuredToa - txTimeSec) * c;
    m_myLastRanges[senderId]    = myMeasuredRange;
    m_myLastRangesLos[senderId] = cond.is_los;

    // -----------------------------------------------------------------------
    // 2. INIZIALIZZAZIONE EKF (al primo contatto con questo sender)
    // -----------------------------------------------------------------------

    if (m_ekfBank.find(senderId) == m_ekfBank.end()) {
        // Inizializziamo dalla GPS dichiarata: è l'unico dato disponibile.
        // L'EKF convergerà rapidamente se il GPS è onesto,
        // oppure divergerà rivelando l'attacco.
        m_ekfBank[senderId].Init(claimedGps);
        m_lastCalcTime[senderId]  = currentTime;
        m_alarms[senderId]        = false;
        m_alarmCounter[senderId]  = 0;
        m_okCounter[senderId]     = 0;
        return; // prima misura: init, poi aspettiamo il secondo pacchetto
    }

    // -----------------------------------------------------------------------
    // 3. PREDICT
    // -----------------------------------------------------------------------

    double dt = currentTime - m_lastCalcTime[senderId];
    if (dt <= 0) return;
    m_ekfBank[senderId].Predict(dt);

    // -----------------------------------------------------------------------
    // 4. COSTRUZIONE VETTORE MISURE
    //
    // Misura diretta (is_direct = true):
    //   anchor_pos = mia posizione GPS (non spoofabile: è la mia)
    //   Il bias nell'EKF compensa offset clock sender-receiver
    //   H(i,6) = 1.0
    //
    // Misure peer (is_direct = false):
    //   anchor_pos = GPS dichiarata del peer k (potenzialmente non fidata)
    //   Il bias tra sender e k è ignoto → H(i,6) = 0.0
    //   R aumentato per compensare l'incertezza aggiuntiva
    // -----------------------------------------------------------------------

    std::vector<EKF::Msmnt> inputData;

    // Misura diretta
    EKF::Msmnt myData;
    myData.anchor_pos   = GetCurrentGpsPosition(); // mia posizione (fidata)
    myData.toa          = measuredToa;
    myData.tx_timestamp = txTimeSec;
    myData.is_direct    = true;
    myData.is_los       = cond.is_los;
    inputData.push_back(myData);

    // Misure peer: nodo k ha misurato la distanza verso senderId
    // e l'ha condivisa nel suo ultimo broadcast
    double maxAge = std::min(2.0, 2.0 * m_swarmSize * m_slotDuration);

    for (uint32_t k = 0; k < m_swarmSize; ++k) {
        if (k == m_id || k == senderId) continue;

        // Verifica disponibilità del range k→sender
        auto rangeIt = m_networkRanges.find(k);
        if (rangeIt == m_networkRanges.end()) continue;
        auto rangeToSender = rangeIt->second.find(senderId);
        if (rangeToSender == rangeIt->second.end()) continue;
        if (rangeToSender->second <= 0.0) continue;

        // Verifica freschezza della misura
        double age = currentTime - m_networkRangeTimes[k][senderId];
        if (age > maxAge) continue;

        // Verifica disponibilità GPS del peer k
        if (!m_lastKnownGps.count(k) || !m_lastKnownTime.count(k)) continue;

        //double peerRange  = rangeToSender->second;
        // Ricostruiamo una TOA fittizia coerente con il modello EKF:
        // toa_fittizio - tx_timestamp_fittizio = peerRange / c
        // Usiamo tx_timestamp del sender come riferimento comune
        // Questo è fisicamente corretto perché:
        //   range_k = c * (toa_at_k - txTimeSec_sender) + bias_k
        // Poiché bias_k è ignoto, lo escludiamo dal modello (H(i,6)=0)
        // e aumentiamo R per assorbire questa incertezza

        Eigen::Vector3d peerGps = m_lastKnownGps[k];
        double peerAge = currentTime - m_lastKnownTime[k];
        if (m_lastKnownVelocity.count(k) && peerAge < 1.0)
        peerGps += m_lastKnownVelocity[k] * peerAge;

        EKF::Msmnt peerData;
        peerData.anchor_pos    = peerGps;
        peerData.is_direct     = false;
        peerData.range         = m_networkRanges[k][senderId]; // range puro in metri
        peerData.is_los        = m_networkRangesLos.count(k) ?
                         (m_networkRangesLos[k].count(senderId) ?
                          m_networkRangesLos[k][senderId] : true) : true;
        // toa e tx_timestamp non servono più per le misure peer
        inputData.push_back(peerData);
    }

    // -----------------------------------------------------------------------
    // 5. UPDATE — almeno 4 misure per localizzazione 3D+bias
    // -----------------------------------------------------------------------

    if ((int)inputData.size() >= 4) {
        m_ekfBank[senderId].Update(inputData);
    }
    m_lastCalcTime[senderId] = currentTime;

    // -----------------------------------------------------------------------
    // 6. RILEVAMENTO SPOOFING con soglia adattiva (Mahalanobis)
    //
    // Invece di una soglia fissa in metri (dipendente dalla geometria),
    // usiamo la distanza di Mahalanobis dell'innovazione EKF.
    // Questa è adimensionale e tiene conto dell'incertezza corrente.
    //
    // Confronto con soglia euclidea fissa:
    //   Prima: if (error > 10)   // 10 m fissi — sbagliato in geometrie cattive
    //   Ora:   if (mahal > 4.5)  // soglia statistica adattiva
    //
    // Chi² con n gradi di libertà, p=0.999:
    //   n=4 → 18.47  (ma usiamo sqrt → ~4.3)
    //   n=7 → 24.32  (sqrt → ~4.9)
    // -----------------------------------------------------------------------

    double mahal  = m_ekfBank[senderId].GetMahalanobisDistance();
    double posStd = m_ekfBank[senderId].GetPositionStdDev();
    Eigen::Vector3d estimatedPos = m_ekfBank[senderId].GetPosition();
    double euclError = (estimatedPos - claimedGps).norm();

    // -----------------------------------------------------------------------
    // STRATEGIA DI ALLARME A DUE LIVELLI INDIPENDENTI:
    //
    // Livello 1 — Mahalanobis sulla SOLA misura diretta:
    //   Misura solo quanto la distanza UWB fisica è inconsistente con il GPS
    //   dichiarato. Non è influenzata dal rumore geometrico dei peer.
    //   Con 1 misura, Mahalanobis ~ |y| / sqrt(R) ~ |errore| / sigma_ranging
    //   Soglia = 5.0 → errore > 5 * sigma_ranging (≈ 1.5m in LOS)
    //
    // Livello 2 — Errore euclideo adattivo:
    //   Backup per casi in cui la Mahalanobis è bassa ma la posizione EKF
    //   diverge chiaramente dal GPS dichiarato.
    //   Soglia = max(8m, 5*posStd) → molto conservativa per evitare falsi positivi
    // -----------------------------------------------------------------------
    double adaptiveThreshold = std::max(8.0, 5.0 * posStd);

    bool suspiciousNow = (mahal  > MAHAL_ALARM_THRESHOLD) ||
                         (euclError > adaptiveThreshold);

    // Freeze iniziale: l'EKF ha bisogno di tempo per convergere
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

    // -----------------------------------------------------------------------
    // 7. DEBUG (solo coppia 1→0)
    // -----------------------------------------------------------------------

    if (m_id == 1 && senderId == 0) {
        std::cout << "t=" << currentTime
                  << " sender=" << senderId
                  << " n_peer=" << inputData.size() - 1
                  << " mahal="   << mahal
                  << " ekf_err=" << (estimatedPos - senderTruePos).norm()
                  << " claimed_err=" << (claimedGps - senderTruePos).norm()
                  //<< " posStd="  << posStd
                  << " alarm="   << m_alarms[senderId]
                  //<< " n_msmnt=" << inputData.size()
                  << std::endl;
    }

    // -----------------------------------------------------------------------
    // 8. LOG CSV
    // -----------------------------------------------------------------------

    if (m_csv && m_csv->is_open()) {
        Eigen::Vector3d recoveredPos = m_alarms[senderId] ? estimatedPos : claimedGps;

        SimulationLogger::LogObservation(
            currentTime, senderId, m_id,
            estimatedPos, claimedGps, senderTruePos,
            m_alarms[senderId], recoveredPos, *m_csv);
    }
}

// ---------------------------------------------------------------------------
// GPS con attacco spoofing (ramp lineare)
// ---------------------------------------------------------------------------

Eigen::Vector3d UwbSecurityApp::GetCurrentGpsPosition() {
    Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
    Eigen::Vector3d gps(mobility->GetPosition().x,
                        mobility->GetPosition().y,
                        mobility->GetPosition().z);

    std::normal_distribution<double> noise_xy(0.0, 0.2); // 20cm rumore xy
    std::normal_distribution<double> noise_z(0.0, 0.4); // 40cm rumore Z

    gps.x() += noise_xy(m_rng);
    gps.y() += noise_xy(m_rng);
    gps.z() += noise_z(m_rng);

    if (m_isMalicious) {
        const double TARGET_OFFSET = 15.0; // [m] offset massimo
        const double RAMP_DURATION = 10.0; // [s] durata rampa
        double elapsed  = Simulator::Now().GetSeconds() - m_attackStartTime;
        double progress = std::min(1.0, std::max(0.0, elapsed / RAMP_DURATION));
        gps.y() += TARGET_OFFSET * progress;
    }
    return gps;
}

// ---------------------------------------------------------------------------
// Vote bitmask: bit=0 se il nodo è considerato malevolo
// ---------------------------------------------------------------------------

uint32_t UwbSecurityApp::GetVoteBitmask() {
    uint32_t mask = 0xFFFFFFFF;
    for (auto const& pair : m_alarms)
        if (pair.second) mask &= ~(1u << pair.first);
    return mask;
}

void UwbSecurityApp::SetActive(bool active) {
    m_isActive = active;
}

void UwbSecurityApp::AddPeer(uint32_t peerId) {
    // Quando entra un nuovo drone, ci assicuriamo che non ci siano vecchi dati in memoria
    m_lastKnownGps.erase(peerId);
    m_lastKnownTime.erase(peerId);
    m_lastKnownVelocity.erase(peerId);
    m_ekfBank.erase(peerId);
    m_alarms[peerId] = false;
    m_alarmCounter[peerId] = 0;
    m_okCounter[peerId] = 0;
}

void UwbSecurityApp::RemovePeer(uint32_t peerId) {
    // Quando un drone esce, cancelliamo il suo EKF e la sua cache
    // per non creare "falsi allarmi" o tenere "fantasmi" nello sciame
    m_lastKnownGps.erase(peerId);
    m_lastKnownTime.erase(peerId);
    m_lastKnownVelocity.erase(peerId);
    m_ekfBank.erase(peerId);
    m_alarms.erase(peerId);
    m_networkRanges.erase(peerId);
    m_networkRangeTimes.erase(peerId);
    m_networkRangesLos.erase(peerId);
}