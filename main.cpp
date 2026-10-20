#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/node-list.h"
#include "ns3/netanim-module.h"

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
#include "EKF.h"
#include "GnssModel.h"
#include "SimulationLogger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

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
    std::string targetsId   = "0";
    std::string csvFileName = "tdma_security_log.csv";
    uint32_t runId = 1;
    uint32_t verbose = 1;
    double qAcc = 2.0;
    std::string ranging = "dstwr";
    std::string gnssMode = "receiver";   // "simple" = modello storico (posizione vera + rumore bianco)
    gnss::Params gnssP;

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
    cmd.AddValue("runId",       "Replica indipendente",              runId);
    cmd.AddValue("verbose",     "Output: 0 silenzioso, 1 eventi, 2 debug", verbose);
    cmd.AddValue("qAcc",        "Rumore di processo EKF (accelerazione)", qAcc);
    cmd.AddValue("ranging",     "dstwr (orologi realistici) oppure toa (modello storico)", ranging);
    cmd.AddValue("gnss",        "receiver (ricevitore simulato) oppure simple (modello storico)", gnssMode);
    cmd.AddValue("gnssCommon",  "GNSS: errore comune per satellite [m]", gnssP.commonSigmaM);
    cmd.AddValue("gnssMultipath","GNSS: multipath individuale [m]", gnssP.multipathSigM);
    cmd.AddValue("gnssCode",    "GNSS: rumore di codice individuale [m]", gnssP.codeNoiseSigM);
    double chi2Thr = 16.27;
    cmd.AddValue("chi2Thr",     "Soglia chi-quadro (3 g.d.l.) con il ricevitore GNSS", chi2Thr);
    uint32_t shareK = 10;
    std::string sharePolicy = "oldest";
    cmd.AddValue("shareK",      "Range condivisi per pacchetto", shareK);
    std::string anchorModel = "consider";
    cmd.AddValue("anchorModel", "Incertezza GPS delle ancore (con --gnss=receiver): fixed | r | consider", anchorModel);
    cmd.AddValue("sharePolicy", "Scelta dei range condivisi: nearest (piu' vicini) | oldest (rotazione)", sharePolicy);
    bool   spoof        = false;
    double spoofDist    = 500.0, spoofAz = -37.0, spoofHeight = 2.0, spoofTime = -1.0;
    double spoofOffset  = 15.0,  spoofDrift = 0.0;
    cmd.AddValue("spoof",       "Spoofer GNSS esterno a singola antenna (richiede --gnss=receiver; sostituisce --targetsId)", spoof);
    cmd.AddValue("spoofDist",   "Distanza orizzontale dell'antenna dal centro dello sciame [m]", spoofDist);
    cmd.AddValue("spoofAz",     "Azimut dell'antenna dal centro dello sciame [gradi, 0 = est, 90 = nord]", spoofAz);
    cmd.AddValue("spoofHeight", "Quota dell'antenna [m]", spoofHeight);
    cmd.AddValue("spoofTime",   "Attivazione dello spoofer [s] (negativo = attackTime)", spoofTime);
    cmd.AddValue("spoofOffset", "Posizione falsa: spostamento in y dal centro dello sciame [m]", spoofOffset);
    cmd.AddValue("spoofDrift",  "Posizione falsa: velocita' di trascinamento in y [m/s]", spoofDrift);
    bool imuOn = true;
    imu::Params imuP;
    cmd.AddValue("imu",           "Sotto cattura: velocita' dai sensori di bordo (IMU/odometria/bussola) nei filtri", imuOn);
    cmd.AddValue("imuVelNoise",   "Sensori di bordo: rumore bianco della velocita' [m/s]", imuP.velNoise);
    cmd.AddValue("imuVelBias",    "Sensori di bordo: bias lento della velocita' [m/s] (tau 30 s)", imuP.velBias);
    cmd.AddValue("imuScale",      "Sensori di bordo: errore di scala (relativo, 1 sigma)", imuP.scale);
    cmd.AddValue("imuHeadingDeg", "Sensori di bordo: errore della bussola [gradi] (tau 120 s)", imuP.headingDeg);
    double ppsSawtoothNs = 10.0, ppsNoiseNs = 1.0;
    cmd.AddValue("ppsSawtoothNs", "PPS: quantizzazione a dente di sega, uniforme +-[ns] (0 = compensata con qErr)", ppsSawtoothNs);
    cmd.AddValue("ppsNoiseNs",    "PPS: rumore gaussiano residuo [ns]", ppsNoiseNs);
    std::string radio = "wifi";
    uwblink::Params linkP;
    cmd.AddValue("radio",      "Radio: wifi (storica, tutti ricevono tutto) | uwb (perdite in funzione della distanza)", radio);
    cmd.AddValue("uwbSensDbm", "Radio UWB: sensibilita' del ricevitore [dBm] (-100 ~85 m al 50%; -90 ~23 m)", linkP.sensDbm);
    double formationScale = 1.0;
    cmd.AddValue("formationScale", "Strada A: scala della formazione (distanze x S, velocita' lineari invariate)", formationScale);
    cmd.Parse(argc, argv);
    UwbSecurityApp::SetVerbosity(verbose);
    EKF::SetDefaultQAcc(qAcc);
    if (sharePolicy != "nearest" && sharePolicy != "oldest") {
        std::cerr << "--sharePolicy deve essere nearest oppure oldest" << std::endl; return 1;
    }
    UwbSecurityApp::SetRangeSharing(shareK, sharePolicy == "oldest");
    UwbSecurityApp::SetImu(imuOn, imuP);
    if (anchorModel == "fixed") UwbSecurityApp::SetAnchorModel(0);
    else if (anchorModel == "r") UwbSecurityApp::SetAnchorModel(1);
    else if (anchorModel == "consider") UwbSecurityApp::SetAnchorModel(2);
    else { std::cerr << "--anchorModel deve essere fixed, r oppure consider" << std::endl; return 1; }
    if (ranging != "dstwr" && ranging != "toa") { std::cerr << "--ranging deve essere dstwr o toa" << std::endl; return 1; }
    UwbSecurityApp::SetRangingDsTwr(ranging == "dstwr");

    RngSeedManager::SetSeed(12345);
    RngSeedManager::SetRun(runId); 

    if (gnssMode != "receiver" && gnssMode != "simple") { std::cerr << "--gnss deve essere receiver o simple" << std::endl; return 1; }
    if (radio != "wifi" && radio != "uwb") { std::cerr << "--radio deve essere wifi oppure uwb" << std::endl; return 1; }
    UwbSecurityApp::SetUwbRadio(radio == "uwb", linkP);
    if (radio == "uwb")
        std::cout << ">>> Radio UWB: EIRP " << uwblink::TxEirpDbm(linkP) << " dBm, sensibilita' " << linkP.sensDbm
                  << " dBm, ricezione al 50% a " << uwblink::Range50(linkP) << " m" << std::endl;
    if (formationScale <= 0.0) { std::cerr << "--formationScale deve essere positivo" << std::endl; return 1; }
    SetFormationScale(formationScale);
    if (formationScale != 1.0) std::cout << ">>> Formazione scalata: S=" << formationScale << std::endl;
    if (spoof && gnssMode != "receiver") { std::cerr << "--spoof richiede --gnss=receiver" << std::endl; return 1; }
    if (gnssMode == "receiver") {
        // Una sola costellazione per simulazione: e' lei a rendere comune l'errore atmosferico tra i droni
        uint64_t gseed = RngSeedManager::GetSeed() * 6364136223846793005ULL + RngSeedManager::GetRun() + 0x6E55ULL;
        UwbSecurityApp::SetGnssConstellation(std::make_shared<gnss::Constellation>(gnssP, gseed));
        if (spoof) {
            // Antenna a terra; posizione falsa = centro dello sciame (nodo 0) + offset, uguale per tutti
            const Eigen::Vector3d center(100.0, 100.0, 50.0);
            double az = spoofAz * M_PI / 180.0;
            auto sp = std::make_shared<gnss::Spoofer>();
            sp->antenna     = Eigen::Vector3d(center.x() + spoofDist * std::cos(az), center.y() + spoofDist * std::sin(az), spoofHeight);
            sp->fakePos0    = center + Eigen::Vector3d(0.0, spoofOffset, 0.0);
            sp->fakeVel     = Eigen::Vector3d(0.0, spoofDrift, 0.0);
            sp->timeOffsetS = 1e-6;
            sp->startS      = spoofTime >= 0.0 ? spoofTime : attackTime;
            UwbSecurityApp::SetGnssSpoofer(sp, "gnss_log.csv");
            UwbSecurityApp::SetPps(ppsSawtoothNs, ppsNoiseNs, "pps_log.csv");
            std::cout << ">>> Spoofer esterno: antenna (" << sp->antenna.transpose() << ") attivo da t=" << sp->startS
                      << " s; l'attacco interno (--targetsId) e' disattivato" << std::endl;
        }
        UwbHeader::SetGnssAccMode(true);
        UwbSecurityApp::SetChi2Threshold(chi2Thr);
    }

    std::cout << "--- Start Simulation Distry MLAT-26 (Decentralized Edition) ---" << std::endl;

    uint32_t totalNodes = setnDrones + setnGuests; 
    std::cout << "Slot totali allocati nel Frame TDMA: " << totalNodes << std::endl;

    std::ofstream csvFile(csvFileName);
    if (csvFile.is_open()) {
        csvFile << "time,sender_id,observer_id,"
                   "est_x,est_y,est_z,claim_x,claim_y,claim_z,"
                   "true_x,true_y,true_z,discrepancy,estimation_error,alarm,"
                   "rec_x,rec_y,rec_z,total_votes,threshold,active_nodes,peer_votes,"
                   "my_vote,suspicious_now,mahal,pos_std,gps_chi2,pairs_ok_frac,swarm_capture,capture_consensus\n";
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

    // ID dei nodi attaccanti, letti una volta sola: servono all'attacco e ai colori di NetAnim
    std::vector<uint32_t> maliciousIds;
    {
        std::stringstream ss(targetsId);
        std::string item;
        while (std::getline(ss, item, ',')) maliciousIds.push_back(std::stoi(item));
        if (spoof) maliciousIds.clear();   // con lo spoofer esterno nessun drone dello sciame mente
    }

    Simulator::Schedule(Seconds(attackTime), [&apps, maliciousIds, totalNodes]() {
        for (uint32_t id : maliciousIds) {
            if (id < totalNodes && apps[id]) {
                apps[id]->SetMalicious(true);
            }
        }
    });

    std::cout << ">>> Configurazione completata. Avvio ns-3..." << std::endl;
    Simulator::Schedule(Seconds(20.0), &PrintProgress, 20.0, setSimTime);
    
    AnimationInterface* anim = nullptr;

    if(netanim) {
        std::cout << ">>> Saving data in NetAnim..." << std::endl;
        anim = new AnimationInterface("esperimento_swarm.xml");
        anim->SetMaxPktsPerTraceFile(5000000);
        
        // Rosso = attaccanti (--targetsId), blu = altri droni base (ID 0..setnDrones-1),
        // giallo = ospiti (ID setnDrones..totalNodes-1)
        for (uint32_t i = 0; i < totalNodes; ++i) {
            anim->UpdateNodeSize(i, 2.0, 2.0);
            bool isAttacker = std::find(maliciousIds.begin(), maliciousIds.end(), i) != maliciousIds.end();
            if (isAttacker) {
                anim->UpdateNodeColor(i, 255, 0, 0);
            } else if (i < setnDrones) {
                anim->UpdateNodeColor(i, 0, 0, 255);
            } else {
                anim->UpdateNodeColor(i, 255, 255, 0);
            }
        }
    }
    std::cout << ">>> Running Simulation..." << std::endl;
    Simulator::Stop(Seconds(setSimTime + 1.0));
    Simulator::Run();
    UwbSecurityApp::PrintLinkStats();
    UwbSecurityApp::PrintPpsStats();
    Simulator::Destroy();
    if (anim) {
        delete anim;
    }
    std::cout << "--- End. ---" << std::endl;
    return 0;
}