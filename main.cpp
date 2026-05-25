#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/node-list.h"

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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

static void PrintProgress(double interval, double totalTime) {
    double now = Simulator::Now().GetSeconds();
    std::cout << ">>> [Progresso] Simulazione a t = " << now 
              << " s (su " << totalTime << " s)..." << std::endl;
              
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval, totalTime);
}

static void LogGroundTruth(std::ofstream* gtFile) {
    if (!gtFile->is_open()) return;

    double now = Simulator::Now().GetSeconds();
    
    // Scorre tutti i nodi fisici della simulazione
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it) {
        Ptr<Node> node = *it;
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        if (mob) {
            ns3::Vector pos = mob->GetPosition();
            *gtFile << now << "," << node->GetId() << "," 
                    << pos.x << "," << pos.y << "," << pos.z << "\n";
        }
    }
    
    // Si auto-richiama ogni 0.5 secondi
    Simulator::Schedule(Seconds(0.5), &LogGroundTruth, gtFile);
}

int main(int argc, char *argv[])
{
    uint32_t setnDrones  = 8;
    uint32_t setnGuests  = 12; 
    double   setSimTime  = 240.0;
    double   attackTime  = 200.0;
    double   setSlot     = 0.005;
    double   setSpeed    = 2.5;
    int      setScenary  = 1;

    std::string targetsId   = "0";
    std::string csvFileName = "tdma_security_log.csv";

    CommandLine cmd;
    cmd.AddValue("setnDrones",  "Numero di droni base nello sciame", setnDrones);
    cmd.AddValue("setnGuests",  "Numero di droni esterni (Ospiti)",  setnGuests);
    cmd.AddValue("setSimTime",  "Durata della simulazione",          setSimTime);
    cmd.AddValue("setSlot",     "Durata slot TDMA",                  setSlot);
    cmd.AddValue("setSpeed",    "Velocità lineare",                  setSpeed);
    cmd.AddValue("attackTime",  "Tempo attacco spoofing",            attackTime);
    cmd.AddValue("setScenary",  "Scenario scelto",                   setScenary);
    cmd.AddValue("targetsId",   "ID target",                         targetsId);
    cmd.AddValue("csvName",     "File CSV",                          csvFileName);
    cmd.Parse(argc, argv);

    std::cout << "--- Start Simulation Distry MLAT-26 (Decentralized Edition) ---" << std::endl;

    // Calcolo totale nodi (il frame TDMA avrà questa dimensione massima)
    uint32_t totalNodes = setnDrones + setnGuests; 
    std::cout << "Slot totali allocati nel Frame TDMA: " << totalNodes << std::endl;

    std::ofstream csvFile(csvFileName);
    if (csvFile.is_open()) {
        csvFile << "time,sender_id,observer_id,"
                   "est_x,est_y,est_z,claim_x,claim_y,claim_z,"
                   "true_x,true_y,true_z,discrepancy,estimation_error,alarm,"
                   "rec_x,rec_y,rec_z,total_votes,threshold,active_nodes,peer_votes\n";
    }

    std::ofstream gtFile("ground_truth.csv");
    if (gtFile.is_open()) {
        gtFile << "time,node_id,true_x,true_y,true_z\n";
    }
    Simulator::Schedule(Seconds(0.0), &LogGroundTruth, &gtFile);

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

    // Assegnazione Traiettorie
    for (uint32_t i = 0; i < setnDrones; ++i) {
        AssignTrajectoryToNode(swarmNodes.Get(i), i, totalNodes, setSimTime, setSpeed, 0.5, setScenary);
    }
    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        AssignGuestTrajectory(swarmNodes.Get(i), i, setSimTime, setSpeed, setScenary, 0.0, -1.0, 0.5);
    }

    // --- SETUP APPLICAZIONI (NESSUN MANAGER CENTRALE) ---
    std::vector<Ptr<UwbSecurityApp>> apps(totalNodes, nullptr);

    // Setup Droni Base (Core)
    for (uint32_t i = 0; i < setnDrones; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        app->SetNodeRole(false); // false = Non è ospite, è un Core node
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        apps[i] = app;
    }

    std::vector<uint32_t> baseIds;
    for (uint32_t i = 0; i < setnDrones; ++i) {
        baseIds.push_back(i);
    }
    for (uint32_t i = 0; i < setnDrones; ++i) {
        apps[i]->InitSlotMap(baseIds);
    }

    // Setup Droni Ospiti
    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        app->SetNodeRole(true); // true = È un Ospite, partirà in stato Fuori Range
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        apps[i] = app;
    }

    // Attacchi (Spoofing)
    Simulator::Schedule(Seconds(attackTime), [&apps, targetsId, totalNodes]() {
        std::vector<uint32_t> maliciousIds;
        std::stringstream ss(targetsId);
        std::string item;
        while (std::getline(ss, item, ',')) maliciousIds.push_back(std::stoi(item));

        for (uint32_t id : maliciousIds) {
            if (id < totalNodes && apps[id]) {
                apps[id]->SetMalicious(true);
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