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
#include "SwarmManager.h"      

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

static void PrintProgress(double interval, double totalTime) {
    double now = Simulator::Now().GetSeconds();
    std::cout << ">>> [Progresso] Simulazione a t = " << now 
              << " s (su " << totalTime << " s)..." << std::endl;
              
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval, totalTime);
}

int main(int argc, char *argv[])
{
    uint32_t setnDrones  = 8;
    double   setSimTime  = 240.0;
    double   attackTime  = 200.0;
    double   setSlot     = 0.005;
    double   setSpeed    = 2.5;
    int      setScenary  = 1;

    std::string targetsId   = "0";
    std::string csvFileName = "tdma_security_log.csv";

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

    SwarmManager tmpMgr;
    tmpMgr.ParseJoin(joinStr);
    tmpMgr.ParseLeave(leaveStr);
    uint32_t nGuestSlots = tmpMgr.CountJoins();

    SwarmManager mgr;
    mgr.Init(setnDrones, nGuestSlots);
    mgr.ParseJoin(joinStr);
    mgr.ParseLeave(leaveStr);
    mgr.PrintEvents();

    uint32_t totalNodes = mgr.GetTotalSlots(); 
    std::cout << "Nodi totali pre-allocati: " << totalNodes << std::endl;

    std::ofstream csvFile(csvFileName);
    if (csvFile.is_open()) {
        csvFile << "time,sender_id,observer_id,"
                   "est_x,est_y,est_z,"
                   "claim_x,claim_y,claim_z,"
                   "true_x,true_y,true_z,"
                   "discrepancy,estimation_error,alarm,"
                   "rec_x,rec_y,rec_z,"
                   "total_votes,threshold,"
                   "active_nodes,peer_votes\n";
    }

    Ptr<UWBChannel> channel = CreateObject<UWBChannel>();
    channel->SetEnvironment("outdoor");

    NodeContainer swarmNodes;
    swarmNodes.Create(totalNodes);

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

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(swarmNodes);

    for (uint32_t i = 0; i < setnDrones; ++i) {
        AssignTrajectoryToNode(swarmNodes.Get(i), i, setnDrones,
                               setSimTime, setSpeed, 0.5, setScenary);
    }

    const std::vector<SwarmEvent>& allEvents = mgr.GetEvents();

    uint32_t tempGuestId = setnDrones;
    for (auto& e : allEvents) {
        if (e.type == SwarmEvent::JOIN) {
            double jt = e.time;

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

    std::vector<Ptr<UwbSecurityApp>> apps(totalNodes, nullptr);

    for (uint32_t i = 0; i < setnDrones; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        app->SetActive(true);
        apps[i] = app;
    }

    // Inizializza la slotMap su tutti i droni base con la lista ordinata degli ID attivi
    {
        std::vector<uint32_t> baseIds;
        for (uint32_t i = 0; i < setnDrones; ++i) baseIds.push_back(i);
        for (uint32_t i = 0; i < setnDrones; ++i)
            apps[i]->InitSlotMap(baseIds);
    }

    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        app->SetActive(false);
        apps[i] = app;
    }

    uint32_t expectedGuestId = setnDrones;

    for (auto& e : allEvents) {
        if (e.type != SwarmEvent::JOIN) continue;

        double jt = e.time;
        uint32_t targetId = expectedGuestId++;

        double lt = -1.0;
        for (auto& le : allEvents) {
            if (le.type == SwarmEvent::LEAVE && le.droneId == targetId) {
                lt = le.time;
                break;
            }
        }
        
        //double approach_time = std::max(0.0, jt - 15.0);

        // Scheduliamo l'accensione e l'inserimento ESATTAMENTE al tempo di join (jt)
Simulator::Schedule(
            Seconds(jt),
            [&mgr, &apps, jt, lt, targetId]() mutable
            {
                uint32_t newId = mgr.AssignId();
                if (newId == UINT32_MAX) return;

                std::cout << ">>> JOIN: drone ID=" << newId
                          << " si inserisce nell'orbita e richiede l'accesso TDMA a t=" << jt << "s"
                          << std::endl;

                // 1. Prepariamo la memoria peer per le distanze
                for (uint32_t id : mgr.GetActiveIds()) {
                    if (id == newId) continue;
                    if (apps[id]) apps[id]->AddPeer(newId);
                    if (apps[newId]) apps[newId]->AddPeer(id);
                }

                // 2. Il nuovo drone "copia" la mappa del frame dal primo drone base disponibile
                // Questo simula il fatto che il drone conosca quanto è "lungo" il treno (m_swarmSize)
                std::map<uint32_t, uint32_t> currentMap;
                for (uint32_t id : mgr.GetActiveIds()) {
                    if (id != newId && apps[id]) {
                        currentMap = apps[id]->GetSlotMap();
                        break;
                    }
                }
                apps[newId]->SetSlotMap(currentMap);

                // 3. Eseguiamo l'Auto-Join Sincronizzato! (Il drone inizia ad ascoltare)
                if (apps[newId]) apps[newId]->JoinSwarm(newId);

                // 4. Gestione dell'uscita (LEAVE) schedulata...
                if (lt > 0) {
                    uint32_t capturedId = newId; 
                    double leave_delay = std::max(0.0, lt - jt); 
                    
                    Simulator::Schedule(
                        Seconds(leave_delay),
                        [&mgr, &apps, capturedId, lt]() {
                            std::cout << ">>> LEAVE: drone ID=" << capturedId
                                      << " richiede il distacco TDMA a t=" << lt << "s" << std::endl;
                            if (apps[capturedId]) apps[capturedId]->ScheduleLeave();
                            mgr.ReleaseId(capturedId);
                        });
                }
            });
    }

    for (auto& e : allEvents) {
        if (e.type == SwarmEvent::LEAVE && e.droneId < setnDrones) {
            uint32_t baseId = e.droneId;
            double lt = e.time;
            
            Simulator::Schedule(Seconds(lt), [&mgr, &apps, baseId, lt]() {
                std::cout << ">>> GUASTO: drone BASE ID=" << baseId
                          << " ha un'avaria radio a t=" << lt << "s"
                          << " — invia goodbye al prossimo slot TDMA." << std::endl;
                
                if (apps[baseId]) apps[baseId]->ScheduleLeave();
            });
        }
    }
    
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

    std::cout << ">>> Configurazione completata. Avvio ns-3..." << std::endl;
    
    Simulator::Schedule(Seconds(20.0), &PrintProgress, 20.0, setSimTime);

    Simulator::Stop(Seconds(setSimTime + 1.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "--- End. ---" << std::endl;
    return 0;
}