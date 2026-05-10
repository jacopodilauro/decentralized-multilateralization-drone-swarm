#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "UwbSecurityApp.h"
#include "Trajectories.h"
#include "UWBChannel.h"
#include "SimulationLogger.h"
#include "SwarmManager.h"      // NUOVO

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

static void PrintProgress(double interval, double totalTime) {
    double now = Simulator::Now().GetSeconds();
    std::cout << ">>> [Progresso] Simulazione a t = " << now 
              << " s (su " << totalTime << " s)..." << std::endl;
              
    // Si auto-schedula per il prossimo step
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval, totalTime);
}

int main(int argc, char *argv[])
{
    // -----------------------------------------------------------------------
    // Parametri base (invariati)
    // -----------------------------------------------------------------------
    uint32_t setnDrones  = 8;
    double   setSimTime  = 240.0;
    double   attackTime  = 200.0;
    double   setSlot     = 0.005;
    double   setSpeed    = 2.5;
    int      setScenary  = 1;

    std::string targetsId   = "0";
    std::string csvFileName = "tdma_security_log.csv";

    // -----------------------------------------------------------------------
    // Nuovi parametri dinamici
    //   --join="10:0,50:0"    tempo:ignored  (ID assegnato dal pool)
    //   --leave="80:2,120:1"  tempo:droneId
    // -----------------------------------------------------------------------
    std::string joinStr  = "";
    std::string leaveStr = "";

    CommandLine cmd;
    cmd.AddValue("setnDrones",  "Numero di droni base nello sciame",      setnDrones);
    cmd.AddValue("setSimTime",  "Durata della simulazione in secondi",    setSimTime);
    cmd.AddValue("setSlot",     "Durata dello slot TDMA in secondi",      setSlot);
    cmd.AddValue("setSpeed",    "Velocità lineare dello sciame",          setSpeed);
    cmd.AddValue("attackTime",  "Tempo in cui il drone viene attaccato",  attackTime);
    cmd.AddValue("setScenary",  "Tipo di scenario scelto",                setScenary);
    cmd.AddValue("targetsId",   "ID del/dei target",                      targetsId);
    cmd.AddValue("csvName",     "Nome del file CSV di log",               csvFileName);
    cmd.AddValue("join",        "Eventi join: 'tempo:0,...'",             joinStr);
    cmd.AddValue("leave",       "Eventi leave: 'tempo:droneId,...'",      leaveStr);
    cmd.Parse(argc, argv);

    std::cout << "--- Start Simulation Distry MLAT-26 (Dynamic Swarm) ---" << std::endl;

    // -----------------------------------------------------------------------
    // SwarmManager: parsing eventi e calcolo slot totali
    // -----------------------------------------------------------------------
    // Prima pass per contare i join (serve sapere quanti slot pre-allocare)
    SwarmManager tmpMgr;
    tmpMgr.ParseJoin(joinStr);
    tmpMgr.ParseLeave(leaveStr);
    uint32_t nGuestSlots = tmpMgr.CountJoins();

    SwarmManager mgr;
    mgr.Init(setnDrones, nGuestSlots);
    mgr.ParseJoin(joinStr);
    mgr.ParseLeave(leaveStr);
    mgr.PrintEvents();

    uint32_t totalNodes = mgr.GetTotalSlots();  // base + ospiti
    std::cout << "Nodi totali pre-allocati: " << totalNodes << std::endl;

    // -----------------------------------------------------------------------
    // CSV
    // -----------------------------------------------------------------------
    std::ofstream csvFile(csvFileName);
    if (csvFile.is_open()) {
        csvFile << "time,sender_id,observer_id,est_x,est_y,est_z,"
                   "claim_x,claim_y,claim_z,true_x,true_y,true_z,"
                   "discrepancy,estimation_error,alarm,"
                   "rec_x,rec_y,rec_z\n";
    }

    // -----------------------------------------------------------------------
    // Canale UWB
    // -----------------------------------------------------------------------
    Ptr<UWBChannel> channel = CreateObject<UWBChannel>();
    channel->SetEnvironment("outdoor");

    // -----------------------------------------------------------------------
    // Nodi ns-3: pre-allochiamo base + ospiti
    // -----------------------------------------------------------------------
    NodeContainer swarmNodes;
    swarmNodes.Create(totalNodes);

    // --- Rete Wi-Fi Ad-Hoc ---
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart", DoubleValue(30.0));
    wifiPhy.Set("TxPowerEnd",   DoubleValue(30.0));
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, swarmNodes);

    InternetStackHelper internet;
    internet.Install(swarmNodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(devices);

    // --- Mobilità ---
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(swarmNodes);

    // 1. Traiettorie droni base (0..setnDrones-1)
    for (uint32_t i = 0; i < setnDrones; ++i) {
        AssignTrajectoryToNode(swarmNodes.Get(i), i, setnDrones,
                               setSimTime, setSpeed, 0.5, setScenary);
    }

    // 2. Traiettorie droni ospiti (PRE-CALCOLATE A t=0 PER EVITARE DEADLOCK)
    const std::vector<SwarmEvent>& allEvents = mgr.GetEvents();

    uint32_t tempGuestId = setnDrones; // Il primo ospite è l'ID 8
    for (auto& e : allEvents) {
        if (e.type == SwarmEvent::JOIN) {
            double jt = e.time;

            // Cerca se l'utente ha scritto un LEAVE per questo specifico ID (es. 8)
            double lt = -1.0;
            for (auto& le : allEvents) {
                if (le.type == SwarmEvent::LEAVE && le.droneId == tempGuestId) {
                    lt = le.time;
                    break;
                }
            }
            
            AssignGuestTrajectory(swarmNodes.Get(tempGuestId), tempGuestId,
                                  setSimTime, setSpeed, setScenary, jt, lt, 0.5);
            tempGuestId++;
        }
    }

    // -----------------------------------------------------------------------
    // Applicazioni
    // -----------------------------------------------------------------------
    std::vector<Ptr<UwbSecurityApp>> apps(totalNodes, nullptr);

    // Droni base: attivi da subito
    for (uint32_t i = 0; i < setnDrones; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        app->SetActive(true);
        apps[i] = app;
    }

    // Droni ospiti: silenziati (in letargo) finché non entrano
    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile); // Setup inserito!
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        app->SetActive(false);
        apps[i] = app;
    }

// -----------------------------------------------------------------------
    // Schedulazione eventi JOIN / LEAVE
    // -----------------------------------------------------------------------
    uint32_t expectedGuestId = setnDrones; // Il primo ospite sarà l'ID 8

    for (auto& e : allEvents) {
        if (e.type != SwarmEvent::JOIN) continue;

        double jt = e.time;
        uint32_t targetId = expectedGuestId++; // L'ID che sappiamo prenderà questo drone

        // Cerca se c'è un LEAVE esplicito per questo ID
        double lt = -1.0;
        for (auto& le : allEvents) {
            if (le.type == SwarmEvent::LEAVE && le.droneId == targetId) {
                lt = le.time;
                break;
            }
        }
        
        double approach_time = std::max(0.0, jt - 15.0);

        Simulator::Schedule(
            Seconds(approach_time),
            [&mgr, &apps, jt, lt, approach_time, targetId]() mutable
            {
                uint32_t newId = mgr.AssignId(); // Coinciderà esattamente con targetId
                if (newId == UINT32_MAX) return;

                std::cout << ">>> APPROACH: drone ID=" << newId
                          << " accende la radio e inizia avvicinamento a t=" << approach_time << "s"
                          << std::endl;

                apps[newId]->SetActive(true);

                for (uint32_t id : mgr.GetActiveIds()) {
                    if (id == newId) continue;
                    if (apps[id]) apps[id]->AddPeer(newId);
                }

                if (jt > approach_time) {
                    Simulator::Schedule(Seconds(jt - approach_time), [newId, jt]() {
                        std::cout << ">>> JOIN: drone ID=" << newId
                                  << " entra fisicamente nell'orbita a t=" << jt << "s"
                                  << std::endl;
                    });
                }

                if (lt > 0) {
                    uint32_t capturedId = newId; 
                    double leave_delay = std::max(0.0, lt - approach_time);
                    Simulator::Schedule(
                        Seconds(leave_delay),
                        [capturedId, lt]() {
                            std::cout << ">>> LEAVE: drone ID=" << capturedId
                                      << " si sgancia dall'orbita a t=" << lt << "s"
                                      << std::endl;
                        });

                    double turn_off_time = lt + 15.0;
                    double turn_off_delay = std::max(0.0, turn_off_time - approach_time);
                    
                    Simulator::Schedule(
                        Seconds(turn_off_delay),
                        [&mgr, &apps, capturedId, turn_off_time]() {
                            std::cout << ">>> SILENZIO RADIO: drone ID=" << capturedId
                                      << " spegne la radio a t=" << turn_off_time << "s"
                                      << std::endl;
                            
                            apps[capturedId]->SetActive(false);
                            for (uint32_t id : mgr.GetActiveIds()) {
                                if (id == capturedId) continue;
                                if (apps[id]) apps[id]->RemovePeer(capturedId);
                            }
                            mgr.ReleaseId(capturedId);
                        });
                }
            });
    }

    // -----------------------------------------------------------------------
    // Schedulazione LEAVE per i droni BASE (Simulazione Guasto/Abbandono)
    // -----------------------------------------------------------------------
    for (auto& e : allEvents) {
        if (e.type == SwarmEvent::LEAVE && e.droneId < setnDrones) {
            uint32_t baseId = e.droneId;
            double lt = e.time;
            
            Simulator::Schedule(Seconds(lt), [&mgr, &apps, baseId, lt]() {
                std::cout << ">>> GUASTO: drone BASE ID=" << baseId
                          << " ha un'avaria radio e scompare dalla rete a t=" << lt << "s"
                          << std::endl;
                
                // Spegne la scheda di rete UWB
                if (apps[baseId]) apps[baseId]->SetActive(false);

                // Notifica gli altri droni di cancellare la sua cache EKF
                // per non usare dati vecchi e sballare le stime
                for (uint32_t id : mgr.GetActiveIds()) {
                    if (id == baseId) continue;
                    if (apps[id]) apps[id]->RemovePeer(baseId);
                }
                
                // NOTA: Non rilasciamo l'ID nel pool, perché gli ID base 
                // non devono essere riassegnati ai droni ospiti esterni!
            });
        }
    }
    
    // -----------------------------------------------------------------------
    // Attacco spoofing (invariato)
    // -----------------------------------------------------------------------
    Simulator::Schedule(Seconds(attackTime), [&apps, targetsId, totalNodes]() {
        std::vector<uint32_t> maliciousIds;
        std::stringstream ss(targetsId);
        std::string item;
        while (std::getline(ss, item, ','))
            maliciousIds.push_back(std::stoi(item));

        for (uint32_t id : maliciousIds) {
            if (id < totalNodes && apps[id]) {
                apps[id]->SetMalicious(true);
                std::cout << ">>> ATTACK ACTIVATED: drone GPS spoofing <"
                          << id << "> starts at t="
                          << Simulator::Now().GetSeconds() << "s <<<" << std::endl;
            } else {
                std::cout << ">>> ERRORE: drone <" << id
                          << "> non esiste o non è attivo!" << std::endl;
            }
        }
    });

    // -----------------------------------------------------------------------
    // Avvio simulatore
    // -----------------------------------------------------------------------
    std::cout << ">>> Configurazione completata. Avvio ns-3..." << std::endl;
    
    // Log di progresso ogni 20 secondi (così capisci subito che lavora!)
    Simulator::Schedule(Seconds(20.0), &PrintProgress, 20.0, setSimTime);

    Simulator::Stop(Seconds(setSimTime + 1.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "--- End. ---" << std::endl;
    return 0;
}