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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

int main(int argc, char *argv[])
{
    uint32_t numDrones = 8;
    double simTime = 240.0;
    double attackTime = 200.0;
    double slotTime = 0.005;    
    double speed = 2.5;
    int SCENARY_TYPE = 1;    

    std::string targetsId = "0";
    std::string csvFileName = "tdma_security_log.csv";
    
    CommandLine cmd;
    cmd.AddValue("numDrones", "Numero di droni nello sciame", numDrones);
    cmd.AddValue("simTime", "Durata della simulazione in secondi", simTime);
    cmd.AddValue("slotTime", "Durata dello slot TDMA in secondi", slotTime);
    cmd.AddValue("speed", "Velocità lineare dello sciame", speed);
    cmd.AddValue("attackTime", "Tempo in cui il drone viene attaccato", attackTime);
    cmd.AddValue("scenaryType", "Tipo di scenario scelto", SCENARY_TYPE);
    cmd.AddValue("targetsId", "ID del/dei target", targetsId);
    cmd.AddValue("csvName", "Nome del file CSV di log", csvFileName);
    cmd.Parse(argc, argv);

    std::cout << "--- Start Simulation Distry MLAT-26 ---" << std::endl;

    std::ofstream csvFile(csvFileName);
    if(csvFile.is_open()) {
        csvFile << "time,sender_id,observer_id,est_x,est_y,est_z,claim_x,claim_y,claim_z,true_x,true_y,true_z,discrepancy,estimation_error,alarm,rec_x,rec_y,rec_z\n";
    }

    Ptr<UWBChannel> channel = CreateObject<UWBChannel>();
    channel->SetEnvironment("outdoor");

    NodeContainer swarmNodes;
    swarmNodes.Create(numDrones);
    
    // --- INSTALLAZIONE RETE (Antenne e IP) ---
    // Installiamo un'antenna Wi-Fi Ad-Hoc standard per far viaggiare i pacchetti
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);

    YansWifiPhyHelper wifiPhy;

    wifiPhy.Set("TxPowerStart", DoubleValue(30.0));
    wifiPhy.Set("TxPowerEnd", DoubleValue(30.0));

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
    // ---------------------------------------------------

    // 4. Configurazione della Mobilità
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(swarmNodes);

    // Assegniamo le rotte matematiche (Waypoint)
    for (uint32_t i = 0; i < numDrones; ++i) {
        AssignTrajectoryToNode(swarmNodes.Get(i), i, numDrones, simTime, speed, 0.5, SCENARY_TYPE);
    }    

    // 5. Installazione dell'Applicazione UWB (Il "Cervello" del drone)
    std::vector<Ptr<UwbSecurityApp>> apps;

    for (uint32_t i = 0; i < numDrones; ++i)
    {
        Ptr<UwbSecurityApp> app = CreateObject<UwbSecurityApp>();
        
        // Passiamo id, totali, canale radio e puntatore al file CSV
        app->Setup(i, numDrones, slotTime, channel, &csvFile); 
        swarmNodes.Get(i)->AddApplication(app);
        
        app->SetStartTime(Seconds(0.0));
        app->SetStopTime(Seconds(simTime));

        apps.push_back(app);
    }

    // 6. Pianificazione dell'Attacco Spoofing sul nodo 0
    /*Simulator::Schedule(Seconds(attackTime), [&apps]() {
        if(apps.size() > 0) {
            apps[0]->SetMalicious(true);
            std::cout << ">>> ATTACK ACTIVATED: drone GPS spoofing <0> starts at t=" << Simulator::Now().GetSeconds() << "s <<<" << std::endl;
        }
    });*/
    
    // 6. Pianificazione dell'Attacco Spoofing Multiplo Dinamico
    Simulator::Schedule(Seconds(attackTime), [&apps, targetsId]() {
        std::vector<uint32_t> maliciousIds;
        std::stringstream ss(targetsId);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            maliciousIds.push_back(std::stoi(item));
        }

        for (uint32_t id : maliciousIds) {
            if (id < apps.size()) {
                apps[id]->SetMalicious(true);
                std::cout << ">>> ATTACK ACTIVATED: drone GPS spoofing <" << id 
                          << "> starts at t=" << Simulator::Now().GetSeconds() << "s <<<" << std::endl;
            } else {
                std::cout << ">>> ERRORE: Hai chiesto di attaccare il drone <" << id 
                          << ">, ma lo sciame ha solo " << apps.size() << " droni!" << std::endl;
            }
        }
    });

    // 7. Avvio del Simulatore
    std::cout << ">>> Configurazione completata. Avvio del motore ns-3..." << std::endl;
    Simulator::Stop(Seconds(simTime + 1.0)); 
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "--- End. ---" << std::endl;
    return 0;
}
