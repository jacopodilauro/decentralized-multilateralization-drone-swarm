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
#include "ConsoleUi.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DistryMlatMain");

static void PrintProgress(double interval, double totalTime) {
    double now = Simulator::Now().GetSeconds();

    if (now > totalTime) {
        return; 
    }
    ui::Tick(now);
              
    Simulator::Schedule(Seconds(interval), &PrintProgress, interval, totalTime);
}

// Inizio dello spoofer, per l'evento ATTACK (stampa soltanto: nessun evento di simulazione in piu')
static double g_spoofStartS = -1.0;
static std::string g_spoofText;

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
    // Avanzamento a schermo agganciato a questo evento periodico gia' esistente (0,5 s simulati)
    if (g_spoofStartS >= 0.0 && now >= g_spoofStartS) {
        ui::Event(g_spoofStartS, "ATTACK", g_spoofText);
        g_spoofStartS = -1.0;
    }
    ui::Tick(now);
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
    std::string anchorVel = "ekf";
    cmd.AddValue("anchorVel", "Velocita' per riportare le ancore all'istante della misura: ekf (storico) | gps (GPS dichiarati)", anchorVel);
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
    double ppsBiasNs = 0.0, ppsTauS = 0.0;
    cmd.AddValue("ppsBiasNs", "PPS: ritardo fisso di ogni ricevitore ~ N(0, sigma) [ns] (0 = nessuno)", ppsBiasNs);
    cmd.AddValue("ppsTauS",   "PPS: costante di tempo della transizione al tempo falsato [s] (0 = immediata)", ppsTauS);
    std::string radio = "wifi";
    uwblink::Params linkP;
    cmd.AddValue("radio",      "Radio: wifi (storica, tutti ricevono tutto) | uwb (perdite in funzione della distanza)", radio);
    cmd.AddValue("uwbSensDbm", "Radio UWB: sensibilita' del ricevitore [dBm] (-100 ~85 m al 50%; -90 ~23 m)", linkP.sensDbm);
    std::string uwbError = "legacy";
    uwberr::Params errP;
    bool uwbLog = false;
    cmd.AddValue("uwbError",      "Errore UWB: legacy (stati LOS indipendenti a ogni messaggio, bit LOS ideale) | real (Fase 5)", uwbError);
    cmd.AddValue("uwbLosDist",    "Errore UWB real: P_LOS(d) = exp(-d / uwbLosDist) [m]", errP.losDist);
    cmd.AddValue("uwbCorrDist",   "Errore UWB real: distanza di correlazione dello stato LOS [m] (0 = indipendente)", errP.corrDist);
    cmd.AddValue("uwbSigmaFloor", "Errore UWB real: rumore per messaggio con segnale forte [m]", errP.sigmaFloor);
    cmd.AddValue("uwbSigmaEdge",  "Errore UWB real: rumore aggiuntivo per messaggio a margine 0 dB [m] (0 = costante)", errP.sigmaEdge);
    cmd.AddValue("uwbPmiss",      "Errore UWB real: probabilita' che un episodio NLOS non sia rilevato", errP.pMiss);
    cmd.AddValue("uwbPfa",        "Errore UWB real: probabilita' che un messaggio LOS sia dichiarato NLOS", errP.pFa);
    cmd.AddValue("uwbLog",        "Log di ogni DS-TWR in uwb_log.csv (verita', misura, stato vero e dichiarato)", uwbLog);
    std::string formation = "legacy";
    double spacing = 12.0, shellSpeed = 1.0;
    bool mixed = false, insiderSpread = false;
    cmd.AddValue("formation",     "Geometria dei droni base: legacy (storica) | shell (guscio sferico a spaziatura costante)", formation);
    cmd.AddValue("spacing",       "Formazione shell: distanza voluta dal drone piu' vicino [m]", spacing);
    cmd.AddValue("shellSpeed",    "Formazione shell: velocita' di rotazione all'equatore [m/s]", shellSpeed);
    cmd.AddValue("mixed",         "Attaccanti interni (--targetsId da --attackTime) anche con lo spoofer esterno (--spoof)", mixed);
    cmd.AddValue("insiderSpread", "Attaccanti interni con direzioni diverse (0 = tutti verso nord, coordinati)", insiderSpread);
    double formationScale = 1.0;
    cmd.AddValue("formationScale", "Strada A: scala della formazione (distanze x S, velocita' lineari invariate)", formationScale);
    cmd.Parse(argc, argv);
    UwbSecurityApp::SetVerbosity(verbose);
    ui::Init(setSimTime, (int)verbose);
    std::string cfgRadio = "Wi-Fi (legacy: every packet received)", cfgUwbErr, cfgScale, cfgSpoof, cfgInsiders, cfgSwarm,
                cfgGeom, cfgTdma;
    EKF::SetDefaultQAcc(qAcc);
    if (sharePolicy != "nearest" && sharePolicy != "oldest") {
        std::cerr << "--sharePolicy must be nearest or oldest" << std::endl; return 1;
    }
    UwbSecurityApp::SetRangeSharing(shareK, sharePolicy == "oldest");
    UwbSecurityApp::SetImu(imuOn, imuP);
    if (anchorModel == "fixed") UwbSecurityApp::SetAnchorModel(0);
    else if (anchorModel == "r") UwbSecurityApp::SetAnchorModel(1);
    else if (anchorModel == "consider") UwbSecurityApp::SetAnchorModel(2);
    else { std::cerr << "--anchorModel must be fixed, r or consider" << std::endl; return 1; }
    if (anchorVel != "ekf" && anchorVel != "gps") { std::cerr << "--anchorVel must be ekf or gps" << std::endl; return 1; }
    UwbSecurityApp::SetAnchorVelGps(anchorVel == "gps");
    if (ranging != "dstwr" && ranging != "toa") { std::cerr << "--ranging must be dstwr or toa" << std::endl; return 1; }
    UwbSecurityApp::SetRangingDsTwr(ranging == "dstwr");

    RngSeedManager::SetSeed(12345);
    RngSeedManager::SetRun(runId); 

    if (gnssMode != "receiver" && gnssMode != "simple") { std::cerr << "--gnss must be receiver or simple" << std::endl; return 1; }
    if (radio != "wifi" && radio != "uwb") { std::cerr << "--radio must be wifi or uwb" << std::endl; return 1; }
    UwbSecurityApp::SetUwbRadio(radio == "uwb", linkP);
    if (radio == "uwb")
        cfgRadio = "UWB, 50% reception at " + ui::Fmt(uwblink::Range50(linkP), 1) + " m (sensitivity "
                   + ui::Num(linkP.sensDbm) + " dBm)";
    if (uwbError != "legacy" && uwbError != "real") { std::cerr << "--uwbError must be legacy or real" << std::endl; return 1; }
    errP.on = (uwbError == "real");
    errP.link = linkP;
    cfgUwbErr = errP.on ? "realistic, P_LOS = exp(-d/" + ui::Num(errP.losDist) + " m), NLOS correlated over "
                              + ui::Num(errP.corrDist) + " m"
                        : "legacy (independent LOS states, ideal LOS flag)";
    if (uwbLog) UwbSecurityApp::SetUwbLog("uwb_log.csv");
    if (formation != "legacy" && formation != "shell") { std::cerr << "--formation must be legacy or shell" << std::endl; return 1; }
    if (formation == "shell" && (setnDrones < 5 || spacing <= 0.0 || shellSpeed < 0.0)) {
        std::cerr << "--formation=shell needs at least 5 base drones, --spacing > 0 and --shellSpeed >= 0" << std::endl; return 1;
    }
    SetFormation(formation == "shell" ? 1 : 0, spacing, shellSpeed);
    if (formationScale <= 0.0) { std::cerr << "--formationScale must be positive" << std::endl; return 1; }
    SetFormationScale(formationScale);
    if (formationScale != 1.0) cfgScale = "S = " + ui::Num(formationScale) + " (distances x S, linear speeds unchanged)";
    ui::Json("formation_scale", ui::Num(formationScale));   // la vecchia riga "Formazione scalata: S=" (aperture_sweep)
    if (spoof && gnssMode != "receiver") { std::cerr << "--spoof needs --gnss=receiver" << std::endl; return 1; }
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
            if (ppsBiasNs < 0.0 || ppsTauS < 0.0) { std::cerr << "--ppsBiasNs and --ppsTauS must be >= 0" << std::endl; return 1; }
            UwbSecurityApp::SetPpsReceiver(ppsBiasNs, ppsTauS);
            // Stessa formattazione della vecchia riga "Spoofer esterno: antenna (...)" letta dagli script
            std::string ax = ui::Num(sp->antenna.x()), ay = ui::Num(sp->antenna.y()), az2 = ui::Num(sp->antenna.z());
            ui::Json("spoofer_antenna_m", "[" + ax + ", " + ay + ", " + az2 + "]");
            ui::Json("spoofer_start_s", ui::Num(sp->startS));
            cfgSpoof = ui::Num(spoofDist) + " m away, azimuth " + ui::Num(spoofAz) + ui::Deg() + ", from t = "
                       + ui::Num(sp->startS) + " s";
            g_spoofStartS = sp->startS;
            g_spoofText = "spoofer on, " + ui::Num(spoofDist) + " m away, azimuth " + ui::Num(spoofAz) + ui::Deg();
        }
        UwbHeader::SetGnssAccMode(true);
        UwbSecurityApp::SetChi2Threshold(chi2Thr);
    }

    uint32_t totalNodes = setnDrones + setnGuests;
    // Stessa formattazione della vecchia riga ">>> Sciame: ..." letta dagli script (raggio, frame, droni base)
    ui::Json("n_base_drones", std::to_string(setnDrones));
    ui::Json("n_guests", std::to_string(setnGuests));
    ui::Json("frame_ms", ui::Num(totalNodes * setSlot * 1000.0));
    cfgSwarm = std::to_string(totalNodes) + " drones (" + std::to_string(setnDrones) + " base, " + std::to_string(setnGuests)
               + " guests), " + formation + " formation";
    if (formation == "shell") {
        ui::Json("shell_radius_m", ui::Num(ShellRadius((int)setnDrones - 1)));
        cfgGeom = "radius " + ui::Fmt(ShellRadius((int)setnDrones - 1), 1) + " m, spacing " + ui::Num(spacing)
                  + " m, rotation " + ui::Num(shellSpeed) + " m/s";
    }
    cfgTdma = "frame " + ui::Num(totalNodes * setSlot * 1000.0) + " ms (" + std::to_string(totalNodes) + " slots of "
              + ui::Num(setSlot * 1000.0) + " ms)";

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
    channel->SetErrorModel(errP);

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
        if (spoof && !mixed) maliciousIds.clear();   // con lo spoofer esterno nessun drone mente (salvo --mixed=1)
    }
    if (!maliciousIds.empty() && (insiderSpread || mixed || formation == "shell")) {
        for (size_t k = 0; k < maliciousIds.size(); k++) {
            double deg = insiderSpread ? 90.0 + 360.0 * k / maliciousIds.size() : 90.0;
            if (maliciousIds[k] < totalNodes && apps[maliciousIds[k]]) apps[maliciousIds[k]]->SetAttackDirection(deg);
        }
    }
    if (!maliciousIds.empty()) {
        std::vector<int> ids(maliciousIds.begin(), maliciousIds.end());
        cfgInsiders = "drone" + std::string(ids.size() > 1 ? "s " : " ") + ui::IdList(ids) + " from t = " + ui::Num(attackTime)
                      + " s, " + (insiderSpread ? "each in its own direction" : "drifting north");
    }

    Simulator::Schedule(Seconds(attackTime), [&apps, maliciousIds, totalNodes]() {
        if (!maliciousIds.empty()) {
            std::vector<int> ids(maliciousIds.begin(), maliciousIds.end());
            ui::Event(Simulator::Now().GetSeconds(), "ATTACK", "insider" + std::string(ids.size() > 1 ? "s " : " ")
                      + ui::IdList(ids) + (ids.size() > 1 ? " start" : " starts") + " declaring a false GNSS position");
        }
        for (uint32_t id : maliciousIds) {
            if (id < totalNodes && apps[id]) {
                apps[id]->SetMalicious(true);
            }
        }
    });

    Simulator::Schedule(Seconds(20.0), &PrintProgress, 20.0, setSimTime);
    
    AnimationInterface* anim = nullptr;

    if(netanim) {
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
    ui::Title(std::string("Distry MLAT-26") + ui::Dot() + "seed " + std::to_string(runId) + ui::Dot()
              + ui::Num(setSimTime) + " s simulated");
    ui::Config("swarm", cfgSwarm);
    if (!cfgGeom.empty()) ui::Config("geometry", cfgGeom);
    if (!cfgScale.empty()) ui::Config("scale", cfgScale);
    ui::Config("TDMA", cfgTdma);
    ui::Config("radio", cfgRadio);
    ui::Config("ranging", ranging == "dstwr" ? "DS-TWR from TDMA broadcasts" : "ToA (legacy model)");
    ui::Config("UWB error", cfgUwbErr);
    ui::Config("GNSS", gnssMode == "receiver" ? "realistic receiver, chi2 threshold " + ui::Num(chi2Thr) : "simple (legacy model)");
    if (!cfgSpoof.empty()) ui::Config("spoofer", cfgSpoof);
    if (!cfgInsiders.empty()) ui::Config("insiders", cfgInsiders);
    if (cfgSpoof.empty() && cfgInsiders.empty()) ui::Config("attack", "none");
    if (netanim) ui::Config("output", "NetAnim trace esperimento_swarm.xml");
    // Avvisi su configurazioni che sappiamo problematiche (solo stampa)
    std::vector<int> liars(maliciousIds.begin(), maliciousIds.end());
    if (!liars.empty() && totalNodes < liars.size() + 5)
        ui::Config("warning", "N >= k + 5 violated (N = " + std::to_string(totalNodes) + ", k = " + std::to_string(liars.size())
                   + "): too few trusted anchors, the system is not expected to hold");
    if (anchorVel == "ekf" && totalNodes >= 30)
        ui::Config("warning", "--anchorVel=ekf is known to diverge with 30 or more drones: use --anchorVel=gps");
    ui::SetScenario(liars, attackTime, g_spoofStartS);
    ui::Json("sim_time_s", ui::Num(setSimTime));
    ui::Json("seed", std::to_string(runId));
    ui::PrintHeader();
    Simulator::Stop(Seconds(setSimTime + 1.0));
    Simulator::Run();
    UwbSecurityApp::PrintLinkStats();
    UwbSecurityApp::PrintPacketStats();
    UwbSecurityApp::PrintPpsStats();
    channel->PrintErrorStats();
    Simulator::Destroy();
    if (anim) {
        delete anim;
    }
    std::vector<std::string> files = { csvFileName, "ground_truth.csv" };   // i file scritti da questa run
    if (spoof) { files.push_back("gnss_log.csv"); files.push_back("pps_log.csv"); }
    if (uwbLog) files.push_back("uwb_log.csv");
    if (netanim) files.push_back("esperimento_swarm.xml");
    files.push_back("summary.json");
    ui::ScenarioSummary(UwbSecurityApp::ConsensusCount(), totalNodes, files);
    ui::Finish("summary.json");
    return 0;
}