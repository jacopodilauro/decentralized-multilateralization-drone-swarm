#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <map>

#include "UwbSecurityApp.h"
#include "Trajectories.h"
#include "UWBChannel.h"
#include "SimulationLogger.h"
#include "SwarmManager.h"      

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

// Mappa per ricordare lo stato di ingresso di ogni drone (Geofence)
std::map<uint32_t, bool> droneInSwarmStatus;

static void PrintProgress(double interval, double totalTime) {
    double now = Simulator::Now().GetSeconds();
    std::cout << ">>> [Progresso] Simulazione a t = " << now 
              << " s (su " << totalTime << " s)..." << std::endl;
              
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval, totalTime);
}

// ==============================================================================
// GEOFENCE RADAR: Controlla spazialmente l'ingresso e l'uscita dei droni
// ==============================================================================
void MonitorGeofence(NodeContainer swarmNodes, SwarmManager* mgr, std::vector<Ptr<UwbSecurityApp>>* apps, uint32_t setnDrones, uint32_t totalNodes) {
    // Definisci le coordinate del rettangolo virtuale
    double MIN_X = 95.0;  double MAX_X = 105.0;
    double MIN_Y = 95.0;  double MAX_Y = 105.0;
    double TRIGGER_DIST = 10.0; // Trigger a 10 metri

    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        Ptr<MobilityModel> mob = swarmNodes.Get(i)->GetObject<MobilityModel>();
        if (!mob) continue;

        ns3::Vector pos = mob->GetPosition();
        double dx = std::max({0.0, MIN_X - pos.x, pos.x - MAX_X});
        double dy = std::max({0.0, MIN_Y - pos.y, pos.y - MAX_Y});
        double distanceToFence = std::sqrt(dx*dx + dy*dy);

        bool isCloseEnough = (distanceToFence <= TRIGGER_DIST);
        bool isCurrentlyInSwarm = droneInSwarmStatus[i];

        // LOGICA DI JOIN (Ingresso Spaziale)
        if (isCloseEnough && !isCurrentlyInSwarm) {
            droneInSwarmStatus[i] = true;
            
            // 1. Notifica il Manager Centrale (usa la funzione che hai aggiunto nell'header!)
            mgr->ScheduleJoin(i);
            
            std::cout << "\n>>> [GEOFENCE] Nodo " << i << " ENTRA a " << distanceToFence << "m! Assegnato ID TDMA: " << i << "\n";
            
            // 2. Peer Setup: aggiorna le memorie TDMA di tutti i droni attivi
            for (uint32_t activeId : mgr->GetActiveIds()) {
                if (activeId == i) continue;
                if ((*apps)[activeId]) (*apps)[activeId]->AddPeer(i);
                if ((*apps)[i]) (*apps)[i]->AddPeer(activeId);
            }

            // 3. Copia la mappa frame dal Master (ID 0)
            std::map<uint32_t, uint32_t> currentMap;
            for (uint32_t activeId : mgr->GetActiveIds()) {
                if (activeId != i && (*apps)[activeId]) {
                    currentMap = (*apps)[activeId]->GetSlotMap();
                    break;
                }
            }
            (*apps)[i]->SetSlotMap(currentMap);

            // 4. Avvia la radio UWB!
            (*apps)[i]->JoinSwarm(i);
            
        } 
        // LOGICA DI LEAVE (Uscita Spaziale)
        else if (!isCloseEnough && isCurrentlyInSwarm) {
            droneInSwarmStatus[i] = false;
            
            std::cout << "\n>>> [GEOFENCE] Nodo " << i << " ESCE dallo sciame (Distanza > 10m)!\n";
            
            if ((*apps)[i]) (*apps)[i]->ScheduleLeave();
            mgr->ScheduleLeave(i); // Rilascia l'ID nel manager
        }
    }

    // Auto-richiamo ciclico ogni 0.1 secondi
    Simulator::Schedule(Seconds(0.1), &MonitorGeofence, swarmNodes, mgr, apps, setnDrones, totalNodes);
}


int main(int argc, char *argv[])
{
    uint32_t setnDrones  = 8;
    uint32_t setnGuests  = 12; // Nuovo parametro che sostituisce i joinStr!
    double   setSimTime  = 240.0;
    double   attackTime  = 200.0;
    double   setSlot     = 0.005;
    double   setSpeed    = 2.5;
    int      setScenary  = 1;

    std::string targetsId   = "0";
    std::string csvFileName = "tdma_security_log.csv";

    CommandLine cmd;
    cmd.AddValue("setnDrones",  "Numero di droni base nello sciame (Core)",  setnDrones);
    cmd.AddValue("setnGuests",  "Numero di droni esterni (Pattugliatori)",   setnGuests);
    cmd.AddValue("setSimTime",  "Durata della simulazione in secondi",       setSimTime);
    cmd.AddValue("setSlot",     "Durata dello slot TDMA in secondi",         setSlot);
    cmd.AddValue("setSpeed",    "Velocità lineare dello sciame",             setSpeed);
    cmd.AddValue("attackTime",  "Tempo in cui il drone viene attaccato",     attackTime);
    cmd.AddValue("setScenary",  "Tipo di scenario scelto",                   setScenary);
    cmd.AddValue("targetsId",   "ID del/dei target",                         targetsId);
    cmd.AddValue("csvName",     "Nome del file CSV di log",                  csvFileName);
    cmd.Parse(argc, argv);

    std::cout << "--- Start Simulation Distry MLAT-26 (Geofence Edition) ---" << std::endl;

    SwarmManager mgr;
    mgr.Init(setnDrones, setnGuests);

    uint32_t totalNodes = mgr.GetTotalSlots(); 
    std::cout << "Nodi totali allocati: " << totalNodes << std::endl;

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

    // --- ASSEGNAZIONE TRAIETTORIE ---
    // Core Drones (Master + Fibonacci)
    for (uint32_t i = 0; i < setnDrones; ++i) {
        AssignTrajectoryToNode(swarmNodes.Get(i), i, totalNodes, setSimTime, setSpeed, 0.5, setScenary);
    }
    
    // Guest Drones (Orbitanti e Pattugliatori)
    // Assegnamo la traiettoria da subito in modo che si muovano costantemente nello spazio
    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        droneInSwarmStatus[i] = false; // Inizializza lo stato a "fuori"
        AssignGuestTrajectory(swarmNodes.Get(i), i, setSimTime, setSpeed, setScenary, 0.0, -1.0, 0.5);
    }

    // --- SETUP APPLICAZIONI ---
    std::vector<Ptr<UwbSecurityApp>> apps(totalNodes, nullptr);

    // Droni Base: Radio accesa da subito
    for (uint32_t i = 0; i < setnDrones; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        app->SetActive(true);
        apps[i] = app;
    }

    // Inizializza la slotMap sui droni base
    {
        std::vector<uint32_t> baseIds;
        for (uint32_t i = 0; i < setnDrones; ++i) baseIds.push_back(i);
        for (uint32_t i = 0; i < setnDrones; ++i)
            apps[i]->InitSlotMap(baseIds);
    }

    // Droni Ospiti: Radio spenta all'inizio (SetActive = false)
    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        app->SetActive(false); 
        apps[i] = app;
    }

    // --- ATTIVAZIONE RADAR GEOFENCE ---
    // Avviamo il controllo spaziale al secondo 1.0 della simulazione
    Simulator::Schedule(Seconds(1.0), &MonitorGeofence, swarmNodes, &mgr, &apps, setnDrones, totalNodes);


    // --- GESTIONE ATTACCHI (SPOOFING) ---
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