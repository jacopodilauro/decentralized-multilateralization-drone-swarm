#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/node-list.h"
#include "ns3/netanim-module.h"
#include "ns3/netsimulyzer-module.h"

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

static void SampleAlarmSeries(
    Ptr<netsimulyzer::XYSeries> series,
    std::vector<Ptr<UwbSecurityApp>>* apps,
    uint32_t setnDrones, double simTime)
{
    double now = Simulator::Now().GetSeconds();
    if (now > simTime) return;

    int total = 0;
    for (uint32_t i = 0; i < setnDrones; ++i)
        if ((*apps)[i]) total += (*apps)[i]->GetActiveAlarmCount();
    series->Append(now, (double)total);

    Simulator::Schedule(Seconds(1.0), &SampleAlarmSeries,
                        series, apps, setnDrones, simTime);
}

static void SampleMahalSeries(
    Ptr<netsimulyzer::XYSeries> series,
    std::vector<Ptr<UwbSecurityApp>>* apps,
    uint32_t setnDrones, double simTime)
{
    double now = Simulator::Now().GetSeconds();
    if (now > simTime) return;

    double drone0Mahal = 0.0;
    
    // Prendiamo il valore ESCLUSIVAMENTE dal Drone-0
    if (!apps->empty() && (*apps)[0]) { 
        drone0Mahal = (*apps)[0]->GetAverageMahalanobis(); 
    }
    
    series->Append(now, drone0Mahal);

    Simulator::Schedule(Seconds(1.0), &SampleMahalSeries,
                        series, apps, setnDrones, simTime);
}

static void SampleActiveNodesSeries(
    Ptr<netsimulyzer::XYSeries> series,
    std::vector<Ptr<UwbSecurityApp>>* apps,
    uint32_t nodeIdx, double simTime)
{
    double now = Simulator::Now().GetSeconds();
    if (now > simTime) return;

    int count = (*apps)[nodeIdx] ? (*apps)[nodeIdx]->GetActiveNodeCount() : 0;
    series->Append(now, (double)count);

    Simulator::Schedule(Seconds(1.0), &SampleActiveNodesSeries,
                        series, apps, nodeIdx, simTime);
}

static void PrintProgress(double interval, double totalTime) {
    double now = Simulator::Now().GetSeconds();

    if (now > totalTime) {
        return; 
    }
    std::cout << ">>> [Progresso] Simulazione a t = " << now 
              << " s (su " << totalTime << " s)..." << std::endl;
              
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval, totalTime);
}

static void LogGroundTruth(std::ofstream* gtFile) {
    if (!gtFile->is_open()) return;

    double now = Simulator::Now().GetSeconds();
    
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it) {
        Ptr<Node> node = *it;
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        if (mob) {
            ns3::Vector pos = mob->GetPosition();
            *gtFile << now << "," << node->GetId() << "," 
                    << pos.x << "," << pos.y << "," << pos.z << "\n";
        }
    }    
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
    int      netanim     = 1;
    int      netsimulyzer= 1;
    int      cube        = 0;

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
    cmd.AddValue("netanim",     "Abilita NetAnim",                   netanim);
    cmd.AddValue("netsimulyzer", "Abilita Netsimulyzer",             netsimulyzer);
    cmd.AddValue("cube",        "Abilita Cubo GEOFENCE",            cube);
    cmd.Parse(argc, argv);

    std::cout << "--- Start Simulation Distry MLAT-26 (Decentralized Edition) ---" << std::endl;

    uint32_t totalNodes = setnDrones + setnGuests; 
    std::cout << "Slot totali allocati nel Frame TDMA: " << totalNodes << std::endl;

    std::ofstream csvFile(csvFileName);
    if (csvFile.is_open()) {
        csvFile << "time,sender_id,observer_id,"
                   "est_x,est_y,est_z,claim_x,claim_y,claim_z,"
                   "true_x,true_y,true_z,discrepancy,estimation_error,alarm,"
                   "rec_x,rec_y,rec_z,total_votes,threshold,active_nodes,peer_votes,clock_bias,ekf_std_dev,true_distance\n";
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
//---------------------------------------------------------------------//
//                              ---- BODY ----                         //

    // Assegnazione Traiettorie
    for (uint32_t i = 0; i < setnDrones; ++i) {
        AssignTrajectoryToNode(swarmNodes.Get(i), i, setnDrones, setSimTime, setSpeed, 0.5, setScenary);
    }
    for (uint32_t i = setnDrones; i < totalNodes; ++i) {
        AssignGuestTrajectory(swarmNodes.Get(i), i, setSimTime, setSpeed, setScenary, 0.0, -1.0, 0.5);
    }

    std::vector<Ptr<UwbSecurityApp>> apps(totalNodes, nullptr);

    // Setup Droni Base (Core)
    for (uint32_t i = 0; i < setnDrones; ++i) {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        app->Setup(i, totalNodes, setSlot, channel, &csvFile);
        app->SetNodeRole(false);
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
        app->SetNodeRole(true);
        swarmNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(setSimTime));
        apps[i] = app;
    }

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
    
    /*
     * NETSIMULYZER
    */
    Ptr<netsimulyzer::Orchestrator> orchestrator = nullptr;
    if (netsimulyzer) {
        orchestrator = CreateObject<netsimulyzer::Orchestrator>("swarm_visualization.json");
        orchestrator->SetTimeStep(MilliSeconds(1000), ns3::Time::MS);

        // --- Configurazione nodi 3D ---
        for (uint32_t i = 0; i < totalNodes; ++i) {
            auto nodeConfig = CreateObject<netsimulyzer::NodeConfiguration>(orchestrator);
            nodeConfig->SetAttribute("Name", StringValue("Drone-" + std::to_string(i)));
            
            nodeConfig->SetAttribute("Model", StringValue("models/quadcopter_uav.obj"));
            nodeConfig->SetAttribute("Scale", DoubleValue(4.0));

            if (i == 0) {
                // Master: rosso
                nodeConfig->SetAttribute("BaseColor",
                netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(255u, 0u, 0u)));         
            } else if (i < setnDrones) {
                // Core: blu
                nodeConfig->SetAttribute("BaseColor",
                    netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(0u, 0u, 255u)));
            } else {
                // Guest: giallo
                nodeConfig->SetAttribute("BaseColor",
                    netsimulyzer::OptionalValue<netsimulyzer::Color3>(netsimulyzer::Color3(255u, 255u, 0u)));
            }
            swarmNodes.Get(i)->AggregateObject(nodeConfig);
        }

        // --- Serie 1: Allarmi attivi totali (rosso) ---
// --- Serie 1: Allarmi attivi totali ---
        auto alarmSeries = CreateObject<netsimulyzer::XYSeries>(orchestrator);
        alarmSeries->SetAttribute("Name", StringValue("Active Alarms (total)"));
        // RIGA COLORE ELIMINATA QUI
        Simulator::Schedule(Seconds(0.0), &SampleAlarmSeries,
                            alarmSeries, &apps, setnDrones, setSimTime);

        // --- Serie 2: Mahalanobis media ---
        auto mahalSeries = CreateObject<netsimulyzer::XYSeries>(orchestrator);
        mahalSeries->SetAttribute("Name", StringValue("Avg Mahalanobis Distance, see by Drone-0"));
        // RIGA COLORE ELIMINATA QUI
        Simulator::Schedule(Seconds(0.0), &SampleMahalSeries,
                            mahalSeries, &apps, setnDrones, setSimTime);

        // --- Serie 3: Nodi attivi visti dal drone 0 ---
        auto nodesSeries = CreateObject<netsimulyzer::XYSeries>(orchestrator);
        nodesSeries->SetAttribute("Name", StringValue("Active Nodes (seen by Drone-0)"));
        // RIGA COLORE ELIMINATA QUI
        Simulator::Schedule(Seconds(0.0), &SampleActiveNodesSeries,
                            nodesSeries, &apps, 0u, setSimTime);

        if (cube) {
        ns3::Rectangle limitiGeofence(70.0, 130.0, 70.0, 130.0);
        auto geofence = CreateObject<netsimulyzer::RectangularArea>(orchestrator, limitiGeofence);
        
        geofence->SetAttribute("Name", StringValue("Area di Sicurezza"));
        }
    }

    /*
     * NETANIM
    */
    AnimationInterface* anim = nullptr;
    if(netanim) {
        std::cout << ">>> Saving data in NetAnim..." << std::endl;
        anim = new AnimationInterface("esperimento_swarm.xml");
        anim->SetMaxPktsPerTraceFile(5000000);
        
        for (uint32_t i = 0; i < totalNodes; ++i) {
            anim->UpdateNodeSize(i, 2.0, 2.0); 
            if (i == 0) {
                anim->UpdateNodeColor(i, 255, 0, 0);
            } else if (i > 0 && i <= setnDrones) {
                anim->UpdateNodeColor(i, 0, 0, 255);
            } else {
                anim->UpdateNodeColor(i, 255, 255, 0);
            }
        }   
    }
    std::cout << ">>> Running Simulation..." << std::endl;
    Simulator::Stop(Seconds(setSimTime + 1.0));
    Simulator::Run();
    Simulator::Destroy();
    if (anim) {
        delete anim;
    }
    std::cout << "--- End. ---" << std::endl;
    return 0;
}