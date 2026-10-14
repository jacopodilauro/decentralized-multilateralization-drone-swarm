# ROADMAP – simulatore realistico e localizzazione dello spoofer

Branch di lavoro: `feature/realistic-simulator` · Stato di partenza: tag `v0-baseline`

> Se apri una nuova conversazione con Claude, incolla questo file: contiene tutto il contesto.

## Obiettivo

1. Rendere il simulatore ns-3 privo di bug e il più fedele possibile alla realtà.
2. Modellare uno **spoofer GNSS esterno** (a terra, singola antenna) che inganna i droni dello sciame.
3. Rilevare l'attacco in modo decentralizzato (incoerenza GNSS ↔ ranging UWB + voto a quorum).
4. **Usare il segnale dello spoofer per localizzarlo**: le differenze di clock bias GNSS tra i droni,
   corrette con la sincronizzazione UWB via TDMA, sono misure TDoA verso lo spoofer.
   Non serve una posizione perfetta: basta una buona direzione e una distanza approssimata.

Modello di misura (drone *i*, satellite falso *j*):
`ρ_ij = |x_f − S_j| + β + |p_i − s| + δ_i` ⇒ `b_i = β + |p_i − s| + δ_i`
⇒ `(b_i − δ_i) − (b_ref − δ_ref) = |p_i − s| − |p_ref − s|` (TDoA, servono ≥ 4 droni).
Limite noto: con base corta (sciame compatto) si stima bene la direzione, male la distanza;
si migliora integrando nel tempo e/o allargando la formazione.

## Regole di lavoro

- Un passo = un commit. Ogni passo dice: **cosa** cambia, **perché**, **come**, **cosa aspettarsi**.
- Nessuna modifica viene consegnata senza essere stata compilata ed eseguita.
- Ogni passo si verifica con almeno uno di: regressione multi-seed, test isolato, più seed, grafico.

## Come si verifica un passo

```bash
# compilazione (alternativa rapida alla cartella scratch/ di ns-3)
NS3_PREFIX=/percorso/ns3 NS3_VER=3.44 tools/build_standalone.sh sim

# 3 seed + metriche aggregate
SIM=./sim tools/run_seeds.sh <etichetta>

# output a terminale: --verbose=0 (silenzioso), 1 (eventi, default), 2 (debug completo)
# ranging: --ranging=dstwr (default, orologi realistici) oppure --ranging=toa (modello storico)

# test isolati (EKF e DS-TWR sempre; header e canale se NS3_PREFIX e' impostato)
tools/run_tests.sh

# confronto con il passo precedente
python3 tools/compare.py runs/<prima>/summary.json runs/<dopo>/summary.json
```

Con la build nella cartella `scratch/` di ns-3, l'eseguibile si trova con:
`find build -type f -executable -name "*distry_mlat26v4*"` (dalla radice di ns-3).

Proprietà verificata in Fase 0: **stesso seed ⇒ output identico bit per bit.**

Attenzione: con `--netanim=1` il CSV differisce a livello di arrotondamento numerico (1 riga su
608.593 nel seed 1, colonna `mahal`, 5a cifra di un valore ~1e-8; metriche identiche). Per gli
esperimenti usare sempre `--netanim=0` (lo fa gia' `run_seeds.sh`).

Riferimento corrente: `reference/step6.2_seed1.json` (1 seed) e `reference/step6.2_seed1-3.json` (3 seed).
Default dal passo 6.2: `--gnss=receiver --sharePolicy=oldest --anchorModel=consider`.
Il comportamento del 4.3 si riottiene con `--gnss=simple --sharePolicy=nearest` (CSV identico byte per byte).
Con il ricevitore GNSS l'errore da guardare e' `errore_rel_*` (rispetto al riferimento del GPS, vedi 6.3);
la calibrazione del test e' in `chi2_media_pre` (attesa 3) e `chi2_oltre_999_pre_pct` (attesa 0,1%).

Metriche tra onesti (dal passo 2.4): `*_onesti_da_onesti_*` escludono anche le righe in cui l'osservatore
e' l'attaccante (che, ingannato dal proprio GPS, vede tutti gli altri fuori posto).
Se un passo aggiunge metriche nuove di proposito, si confronta con `--ignora-nuove`.
Se un passo cambia i numeri di proposito, si usa `--tol` e si confronta la differenza con la
deviazione standard tra seed (vedi passo 1.4).

## Numeri di riferimento (v0-baseline, ns-3.44, seed 1–3, default)

La colonna `alarm` del CSV è l'**allarme collettivo** (voti ≥ soglia), non il sospetto individuale.

| Metrica | Media | Min – Max |
|---|---|---|
| Latenza di rilevamento sul nodo 0 (mediana tra osservatori) | 2,40 s | 2,10 – 2,60 s |
| Osservatori che rilevano / che vedono il nodo 0 | 7 / 7 | – |
| Allarmi collettivi prima dell'attacco (falsi allarmi) | 1,36 % | 0,62 – 1,85 % |
| **Allarmi collettivi su nodi onesti dopo l'attacco** | **61,0 %** | 60,1 – 62,3 % |
| Errore EKF nodi base, mediana / p95 | 0,45 / 1,26 m | – |
| Errore EKF ospiti, mediana / p95 | 0,72 / 1,65 m | – |

## Fasi

- [x] **Fase 0 – Infrastruttura**: script di build, metriche, confronto, esecuzione multi-seed; questo file.
      Unica modifica al simulatore: `#include <iomanip>` mancante in `UwbSecurityApp.cpp`.
- [x] **Fase 1 – Bug che non cambiano il modello** (le metriche devono restare identiche, salvo dove previsto)
  - [x] Salvare nel CSV anche il sospetto individuale: nuove colonne `my_vote`, `suspicious_now`,
        `mahal`, `pos_std` (in fondo, le 22 esistenti sono identiche byte per byte)
  - [x] Variabili non inizializzate (`m_isGuest`, `m_macState`, `m_listenCounter`, `m_chosenSlot`, `m_clockOffset`);
        rimossi `m_voteBitmask`, `m_recentLeaves`
  - [x] Colori NetAnim: rosso = attaccanti reali (`--targetsId`), blu = base, giallo = ospiti
        (prima il drone 8 era blu e il rosso era sempre sul nodo 0)
  - [x] Header dei range condivisi: ora solo coppie (ID, range), max 10; la dimensione non dipende
        piu' dall'ID massimo (prima: 240 KB per un solo range verso l'ID 60000). Test: `tests/test_header.cc`
  - [x] Codice morto rimosso (`PrintTerminalDashboard`, `GetVoteBitmask`, `AddPeer`, `AddPeerSlot`,
        `GetFirstAvailableSlot`, dichiarazioni senza corpo); stampe di debug a istanti fissi sostituite
        dall'opzione `--verbose` (0 silenzioso, 1 eventi = default, 2 debug)
  - [x] `viwer.py`: geofence disegnato come lo calcola il codice (2D, ingresso 10 m, uscita 14 m);
        pannello del consenso con voti individuali e soglia di quorum reale (prima: allarmi collettivi
        contro un 50% inesistente, che mostrava "non in protezione" anche ad allarme scattato);
        opzioni `--csv`, `--target`, `--save`; terminatori di riga Unix
- [ ] **Fase 2 – Errori logici che falsano i risultati**
  - [x] 2.0 Rumore del canale: un generatore per collegamento (prima uno solo condiviso, quindi ogni
        modifica rimescolava il rumore di tutti). Test: `tests/test_channel.cc`
  - [x] 2.1 Ospiti fuori dal geofence non elaborano piu' i pacchetti (prima stimavano e scrivevano
        nel CSV da t=0). Le loro righe abbassavano l'errore EKF dei base (p95 0,67 m contro 1,25 m):
        ora p95 1,31 m, piu' onesto
  - [x] 2.2 Compensazione del ritardo dei range condivisi: ogni range porta la sua eta' e l'EKF lo
        confronta con le posizioni all'istante della misura. Causa del raddoppio dell'errore con gli
        ospiti (7.5 m/s x 0.12 s ~ 0.9 m di errore sistematico). Corretto anche: i droni condividevano
        per sempre i range verso droni usciti. Test: `tests/test_ekf.cc`, `tests/test_header.cc`
  - [x] 2.3 Rumore di processo EKF `q_acc` da 0.5 a 2 (opzione `--qAcc`): con 0.5 il filtro non
        segue le curve degli ospiti (3.75 m/s^2) e la compensazione del ritardo peggiorava la loro stima.
        Sweep 0.5-5: salto tra 0.5 e 1-2, poi andamento graduale
  - [x] 2.4 Avvelenamento delle ancore: un drone sotto allarme collettivo non viene piu' usato come
        ancora. Falsi allarmi tra onesti dopo l'attacco 60% -> 0%, errore onesti p95 10,8 -> 0,99 m.
        Esperimenti: oracolo (mai il nodo 0) = stesso risultato; gating sui residui da solo 14,8%
  - [x] 2.5 Join TDMA: lo slot "scelto" non veniva usato (si trasmette sempre nello slot = ID), le
        collisioni erano finte e le mappe incoerenti: 219.056 voci sbagliate e 3.833 voti attribuiti al
        drone sbagliato (seed 1). Ora TDMA statico slot = ID: 0 voci incoerenti, 0 voti sbagliati
  - [x] 2.5 Uscita dal geofence che scattava due volte: dopo il goodbye la macchina a stati si ferma
  - [x] Allarme collettivo mai usato: ora esclude il drone sospetto dalle ancore (2.4)
  - [x] 2.6 Mahalanobis tolta dalla decisione (resta nel CSV): causava ~70% dei sospetti falsi e 0% dei
        rilevamenti, perche' il residuo della misura diretta e' assorbito dal bias di clock. Sospetti
        istantanei pre-attacco 1,31% -> 0,40%; rilevamento identico
  - [ ] `setSpeed`, `setScenary`, `t_join`, `t_leave` passati ma ignorati da `Trajectories.cpp`
  - [ ] Voti non autenticati: un attaccante potrebbe accusare droni onesti per farli escludere
        come ancore (da trattare nel threat model)
  - [x] Falsi allarmi residui (ospite 19 e altri): causati dagli offset degli orologi. Il range k->j contiene
        (offset_k - offset_j) c e il filtro tratta i range condivisi come senza bias: tutti i range verso j
        hanno lo stesso errore fisso. Correlazione offset/errore di stima -0,95 su 36 ospiti (3 seed);
        oracolo senza offset: falsi allarmi pre-attacco 0%, errore base mediano 0,39 -> 0,25 m.
        Soluzione: Fase 4 (orologi realistici + DS-TWR), anticipata
  - [x] Test chi-quadro sulla discrepanza (stima - GPS) con la covarianza del filtro (6.2, con il
        ricevitore GNSS; con `--gnss=simple` resta la soglia max(2 m, 3,5 x pos_std))
  - [x] Ogni drone condivideva solo i 10 range piu' vicini: con 20 droni quelli periferici non sono tra i
        10 piu' vicini di nessuno, ricevono solo il range diretto (< 4 misure) e il filtro diverge
        (seed 1: nodo 3 senza range condivisi a ~77-90 s e ~140-152 s, pos_std ~48 m, errore fino a 32 m;
        nascosto dal p95). Ora rotazione (`--sharePolicy=oldest`, default; `--shareK`, default 10): ogni
        pacchetto porta i K range condivisi meno di recente. Oracolo con tutti i range (seed 1): base p99
        0,68 m; rotazione 0,74 m; prima 5,54 m. Vedi passo 6.2a
  - [ ] `q_acc` unico per droni lenti e veloci: valutare rumore di processo adattivo per drone
  - [ ] Ricontrollare la sensibilita' con attacchi piu' lenti/sottili dopo l'aumento di `q_acc`
Ordine dei lavori da qui (deciso il 5 ottobre 2026): Fasi 6 -> 7 -> 8 -> 9 (contributo principale),
poi Fasi 3 + 5 insieme (radio e canale), poi Fase 10. Le figure finali del paper solo dopo le Fasi 3 e 5.

- [ ] **Fase 3 – Radio realistica** (portata UWB, perdita pacchetti vs distanza, airtime) al posto del Wi-Fi a 30 dBm;
      valutare qui un'allocazione dinamica degli slot con collisioni vere
- [ ] **Fase 4 – Clock realistici** (offset, deriva ppm, jitter) e ranging dai timestamp dei pacchetti (anticipata)
  - [x] 4.1 `ClockModel.h` (contatore 40 bit, tick 15,65 ps, offset casuale, skew +-20 ppm, jitter) e
        `DsTwr.h` (DS-TWR asimmetrico, differenze modulo 2^40). Test `tests/test_dstwr.cc`:
        fermi 1,2 mm RMS; ToA a una via con gli stessi orologi >= 468 km di errore; azzeramento del
        contatore gestito. In movimento il risultato e' la distanza all'istante del MESSAGGIO CENTRALE
        (2-3 mm anche a 15 m/s); riferita a meta' intervallo sbaglierebbe fino a 33 cm.
        Costo: eta' alla condivisione = 1 frame esatto (oggi in media mezzo frame), +0,05 s
  - [x] 4.2 Timestamp nei pacchetti (solo in modalita' DS-TWR): seq 7 bit, timestamp di trasmissione 40 bit,
        per ogni vicino ID + seq + bit LOS + istante di ricezione (8 byte). Con 20 droni 132 -> 291 byte
  - [x] 4.3 DS-TWR nel simulatore, ora predefinito (`--ranging=dstwr`; il modello storico resta con
        `--ranging=toa`, identico byte per byte al passo 2.6). Distanze verificate contro la verita':
        LOS errore medio 0,03 cm, dev. std 7,1 cm. Su 3 seed rispetto al 2.6: errore base p95 1,01 ->
        0,77 m, falsi allarmi collettivi pre-attacco 0,19% -> 0,034%, rilevamento invariato, risultati
        non piu' dipendenti dagli orologi del seed
  - [ ] Il bit LOS dei timestamp e' ideale (viene dal canale): un rilevatore NLOS reale sbaglia a volte
  - [ ] Scalabilita': il pacchetto cresce di 8 byte per vicino; con sciami grandi limitare a k vicini
  - [ ] Falsi allarmi residui (4.3): ospite 19 nel seed 1, drone base 3 nel seed 3 (episodi brevi)
  - [ ] Offset tra gli orologi dai timestamp DS-TWR (servono per localizzare lo spoofer)
  - [ ] Su droni reali: collegare orologio GNSS e orologio UWB (impulso PPS registrato dal chip UWB)
- [ ] **Fase 5 – Errore UWB realistico** (LOS/NLOS correlato nello spazio e nel tempo)
- [ ] **Fase 6 – Ricevitore GNSS** (satelliti, pseudodistanze, minimi quadrati per posizione e clock bias)
  - [x] 6.1 `GnssModel.h`: costellazione condivisa (errore comune per satellite, Gauss-Markov 1,5 m / 600 s),
        ricevitore per drone (multipath 0,4 m / 10 s, codice 0,2 m, orologio +-1 ms e +-0,5 ppm), minimi
        quadrati per posizione e clock bias. Test `tests/test_gnss.cc`: esatto senza errori; assoluto orizz.
        2,6 m (p95 4,7), relativo tra droni a 20 m 0,70 m (p95 1,6), clock bias 1,3 m. Opzione
        `--gnss=receiver` (default ancora `simple`, identico al 4.3); parametri `--gnssCommon/Multipath/Code`
  - [x] 6.2 Test chi-quadro calibrato: d2 = e' S^-1 e, e = stima - GPS dichiarato, soglia `--chi2Thr` 16,27
        (3 g.d.l., 99,9%), colonna `gps_chi2`. Cause delle code trovate con esperimenti e test isolati:
        (1) divergenze dei droni periferici dai "10 range piu' vicini" (passo 6.2a, rotazione);
        (2) errore GPS delle ancore lento (multipath 10 s) trattato dal filtro come bianco: test NEES con moto
        coerente col modello, ancore Gauss-Markov 10 s -> d2 medio 10,0 e 16,4% oltre soglia; ancore bianche
        -> 3,00. Correzione `--anchorModel=consider` (default): R_i = sigma2_uwb + u' Sigma_ancora u per ogni
        misura e covarianza "consider" del bias da ancore G D G' (G = minimi quadrati pesati) sommata nel
        test: d2 2,43, 0% oltre soglia. Ablazioni: `fixed` (6.2 iniziale, fattore 2), `r` (solo R_i);
        (3) hAcc/vAcc perdono l'anisotropia del GPS (seed 1: sigma x 0,62, y 0,34 m, hAcc 0,47): su 30
        costellazioni diag(h,h,v) -> 0,44% oltre soglia, covarianza piena -> 0,08%. Il pacchetto porta ora
        sigma E/N/U (cm) + 3 correlazioni (1/127): 9 byte (prima 4).
        Varianze UWB dal modello di canale: LOS 0,01 m2, NLOS 0,36 m2; il bit NLOS dei range condivisi viaggia
        nel bit alto del campo range (0 byte; prima il ricevente lo poneva sempre LOS).
        3 seed (receiver, rispetto al 6.2 iniziale): d2 medio 2,67 -> 2,76 +-0,14; oltre soglia 0,54 -> 0,10%
        +-0,05 (atteso 0,1%); errore relativo base p95 2,69 -> 1,20 m, ospiti 1,96 -> 1,16 m; FA pre-attacco
        individuali 0,36 -> 0%, collettivi 0,60 -> 0%; latenza 3,57 -> 3,04 s; rilevamento 7/7.
        Test: `test_ekf` [2] (NEES), `test_gnss` [6] (covarianza), `test_header` (bit LOS, covarianza)
  - [ ] FA collettivi tra onesti dopo l'attacco 0,80 +-1,39% (solo seed 1, 2,41%): l'oracolo senza attacco
        ne riproduce 2,18 punti (nodi 6 e 7 a 210 s) -> escursioni reali del GPS del mittente, viste da tutti
        gli osservatori insieme (il quorum non le filtra): da classificare come allarmi di integrita' GNSS.
        Il residuo legato all'attacco (~0,2%, nodo 2 a 200-205 s) e' probabilmente avvelenamento delle ancore
        prima del rilevamento: da verificare con un oracolo
  - [x] (vecchia nota 6.2) FA esplosi con `--gnss=receiver` e soglia max(2 m, 3,5 pos_std): risolto dal test
        chi-quadro calibrato
  - [~] 6.3 Con il GPS realistico lo sciame si localizza nel riferimento del GPS (spostato dell'errore
        comune): l'errore rispetto alla posizione vera non e' piu' la metrica giusta per la localizzazione.
        Fatto (6.2): `metrics.py` calcola `errore_rel_*` (errore comune stimato ogni 0,1 s come mediana su
        tutti i mittenti di GPS - vero) ed `errore_ekf_*_p99_m`. Da fare: usarle nel viewer e nelle figure
- [ ] **Fase 7 – Spoofer esterno** (posizione, attivazione, cattura dei droni)
  - [x] 7.1 Modello fisico in `GnssModel.h` (`Spoofer`, `Receiver::SetSpoofer`): antenna a terra, satelliti
        finti per una posizione falsa x_f (anche in movimento) e un offset di tempo beta; ritardo |p_i - s|
        uguale su tutti i canali + multipath del collegamento (IPOTESI 0,5 m / 10 s). Nel `Fix`: `spoofed` e
        `ppsErrorS` = bias stimato - bias vero = beta + (|p_i - s| + m_i)/c: il PPS scatta in ritardo della
        distanza dallo spoofer, senza l'orologio del ricevitore. Test `test_gnss` [7]: tutti i droni vedono
        la stessa x_f (scarto max 1,23 m), differenze di PPS = differenze di distanza (RMS 1,00 m), esatto
        senza rumore. [8] informativo, spoofer a 500 m, 60 s di PPS, offset UWB ideali (mediana): raggio 10 m
        -> azimut 0,7-2,3 gradi, distanza non determinabile; raggio 50 m -> 0,2-0,5 gradi, 47-156 m
        (jitter PPS 0-20 ns). Simulatore invariato (nessuno spoofer)
  - [x] 7.2 Spoofer nel simulatore: `--spoof=1` (richiede `--gnss=receiver`, disattiva `--targetsId`),
        `--spoofDist/Az/Height` (antenna, default 500 m, -37 gradi, 2 m), `--spoofTime` (default `attackTime`),
        `--spoofOffset/Drift` (posizione falsa = centro dello sciame + offset in y, trascinamento in m/s).
        Log `gnss_log.csv` (un fix per riga: catturato, GPS, verita', errore PPS in m). Strumento
        `tools/spoofer_view.py`: cattura, reazione del sistema, localizzazione dai PPS (geometria e
        sincronizzazione IDEALI). Seed 1, spoofer a 500 m attivo a 100 s: 20/20 droni catturati in 0,1 s,
        dispersione delle posizioni GPS 0,66 m (vere 13,4 m); sistema attuale: allarmi collettivi 0 -> 84,5%,
        chi2 mediano 2,4 -> 162, pos_std 0,35 -> 2,37 m (segnala tutti: pensato per pochi bugiardi).
        PPS 1 Hz senza jitter: direzione 0,2-0,6 gradi gia' dopo 5-10 s; distanza -246 m a 5 s, +42 m a 120 s
        (apertura dello sciame 32 -> 88 m grazie al moto degli ospiti); jitter 20 ns: direzione stabile da ~20 s.
        Senza `--spoof` il simulatore e' identico al 6.2
  - [ ] Jitter del PPS e quantizzazione (qErr) da schede tecniche reali; offset UWB tra droni dai timestamp DS-TWR
- [ ] **Fase 8 – Rilevamento dello spoofer esterno** con il consenso esistente
  - [x] 8.1 Cattura dello sciame: ogni drone, una volta per frame, confronta per ogni coppia di ALTRI droni la
        distanza tra i GPS dichiarati (riportati all'istante del range con la velocita' dei GPS dichiarati su ~1 s)
        con il range UWB condiviso; coppia coerente se lo scarto e' entro 4 sigma (GPS individuale proiettato +
        UWB 0,36 m2 + velocita' 2 m/s x tempo). Cattura se meno del 25% delle coppie e' coerente (>= 6 coppie)
        per 10 frame; uscita con 10 frame sopra soglia. Uno spoofer a singola antenna fa vedere a tutti lo stesso
        punto: quasi nessuna coppia e' coerente; pochi droni bugiardi rendono incoerenti solo le loro coppie.
        Colonne CSV `pairs_ok_frac`, `swarm_capture`; metriche `cattura_*`, `coppie_coerenti_*`. Solo
        rilevamento: i filtri non cambiano (metriche esistenti identiche al 6.2).
        Risultati (3 seed): nodo 0 bugiardo -> 0% false catture prima e dopo, coppie coerenti 100% / 81%, minimo
        senza attacco 98,8%; spoofer a 100 s -> 20/20 droni in 0,95 s (= isteresi), 100% del tempo, coppie
        coerenti 1-8% (le coppie di droni a meno di ~5 m restano compatibili). Spoofer a 200 s (8 droni): 8/8 in 1,0 s.
        Errore di progetto trovato e corretto: la prima versione usava la velocita' degli EKF, che sotto cattura
        divergono (30-80 m/s) e rendevano "coerenti" a caso il 25-33% delle coppie. Principio: il test di
        cattura non deve dipendere da stati stimati con le ancore GPS
  - [x] 8.2a Consenso sulla cattura: ogni drone annuncia il proprio stato (bit 1 del byte di imLeaving, 0 byte in
        piu'); sciame catturato se almeno 2/3 dei droni attivi (me compreso, annunci <= 1 s) lo annunciano. Sotto
        consenso: niente caccia al bugiardo (nessun sospetto) e nessun aggiornamento dei filtri con ancore GPS
        (solo predizione). Colonna `capture_consensus`, metriche `consenso_cattura_*`.
        Seed 1, spoofer a 100 s: consenso di 20/20 droni in 1,05 s, 100% del tempo, mai interrotto; allarmi
        "bugiardo" sugli onesti 83% -> 1,4% (transitorio prima del consenso). Il comportamento caotico sparisce:
        build normale e build con contrazione FMA ora danno metriche identiche (prima 83,3% contro 84,5%).
        Costo atteso: senza ancore i filtri solo predicono, pos_std mediana 225 m dopo la cattura.
        Senza spoofer: metriche identiche al 6.2, consenso 0%
  - [x] 8.2b Sotto cattura, localizzazione solo relativa con UWB. Istantanee dei filtri ogni secondo SOLO senza
        alcun sospetto; al consenso si riparte dalla piu' vecchia (1-2 s prima dell'attacco). Ancore = mie stime degli
        altri droni (con la loro covarianza) invece dei GPS falsi; un filtro su me stesso (dal mio ultimo GPS buono,
        aggiornato con le distanze dirette) fa da ancora per le misure dirette. I droni che entrano durante la cattura
        non vengono inizializzati (limite). Metrica di FORMA in `metrics.py`: stime allineate alla verita' con la
        miglior rototraslazione (Kabsch), residuo RMS; a parte la deriva del riferimento (traslazione, rotazione).
        Seed 1, spoofer a 100 s: forma mediana 0,22 m / p95 1,59 m (prima dell'attacco, con GPS: 0,54 / 0,94 m;
        8.2a: 645 / 1.475 m); pos_std 0,18 m. MA il riferimento scivola: traslazione 340 m, rotazione 125 gradi
        (mediane). Fisica: posizione e orientamento dell'insieme non sono osservabili da sole distanze; la velocita'
        comune fa un random walk (~t^1,5). Senza spoofer: metriche identiche al 6.2
  - [ ] 8.2c Ancorare il riferimento sotto cattura con i sensori di bordo indipendenti dal GPS (velocita' da IMU/
        odometria visiva, direzione da bussola), da condividere nel pacchetto: serve per dare allo spoofer una
        direzione nel mondo (Fase 9)
  - [x] Nota di metodo (7.2), risolta dall'8.2a: sotto cattura il sistema era caotico; tra macchine diverse le metriche dello
        scenario con spoofer differiscono (es. allarmi 83,3% contro 84,5%) per arrotondamenti di 1e-10 (verificato
        ricompilando con contrazione FMA). Confronti tra macchine: con tolleranza e su piu' seed
- [ ] **Fase 9 – Localizzazione dello spoofer** (istantanea ai minimi quadrati + EKF nel tempo)
- [ ] **Fase 10 – Campagna di esperimenti** per le figure del paper

## Diario

| Data | Fase | Commit | Note |
|---|---|---|---|
| 2026-09-30 | 0 | infrastruttura | Riproducibilità verificata; baseline su 3 seed |
| 2026-09-30 | 0 | – | Ambiente di Jacopo (ns-3-dev) identico a ns-3.44: 26/26 metriche uguali |
| 2026-09-30 | 1.1 | log individual votes | Falsi voti individuali su onesti post-attacco: 58,2 % (seed 1–3). L'avvelenamento delle ancore agisce già nel singolo EKF |
| 2026-09-30 | 1.2 | clean up dead code, add --verbose | CSV identico byte per byte; output da ~5.000 a 212 righe (livello 1) |
| 2026-09-30 | 1.3 | fix netanim colors | Colori corretti; scoperto che NetAnim altera il CSV a livello di arrotondamento |
| 2026-09-30 | 1.4 | send only the shared ranges | Contenuto dei pacchetti identico (prova con pacchetti a 256 byte fissi: CSV identici, seed 1-2). Il CSV reale cambia solo per il tempo di trasmissione Wi-Fi: metriche entro 0,02 deviazioni standard tra seed |
| 2026-09-30 | 1.5 | fix the viewer | Il vecchio pannello del consenso diceva il contrario del codice; scoperti gli ospiti che osservano da fuori geofence |
| 2026-10-01 | 2.0 | per-link channel rng | Statistiche equivalenti (entro la variabilita' tra seed) |
| 2026-10-01 | 2.1 | guests out of range ignore packets | Righe dei base prima del primo ingresso identiche (prova del disaccoppiamento 2.0) |
| 2026-10-01 | 2.2 | compensate the delay of shared ranges | Base p95 1,31 -> 0,90 m; ospiti peggiorati (q_acc troppo basso) |
| 2026-10-01 | 2.3 | raise ekf process noise | Rispetto a 2.1: base p95 1,31 -> 1,01 m, ospiti mediana 0,72 -> 0,61 m, FA pre-attacco 1,38% -> 0,25%, rilevamento invariato |
| 2026-10-01 | 2.4 | stop using alarmed drones as anchors | FA onesti->onesti post 60% -> 0%; pre-attacco e rilevamento invariati |
| 2026-10-01 | 2.5 | static tdma slots, single leave | Collisioni finte 9 -> 0, uscite avviate 24 -> 12, voti mal attribuiti 3.833 -> 0; metriche invariate |
| 2026-10-01 | 2.6 | drop mahalanobis from the decision | Sospetti istantanei pre-attacco 1,31% -> 0,40%; allarmi e rilevamento quasi invariati |
| 2026-10-02 | 4.1 | add uwb clock model and ds-twr | Test isolato superato; nessuna modifica al simulatore |
| 2026-10-02 | 4.2-4.3 | ds-twr ranging in the simulator | Orologi realistici; FA collettivi pre-attacco -82%, errore base p95 -24% |
| 2026-10-05 | – | roadmap order, ignore paper notes | Note private per il paper in PAPER_NOTES.md (non su GitHub) |
| 2026-10-05 | 6.1 | add gnss receiver model (off by default) | Ricevitore testato; attivo rivela che la soglia di sospetto non regge un GPS realistico |
| 2026-10-05 | 6.2 (parziale) | chi-square gnss test (receiver mode) | Media calibrata; code dal multipath correlato delle ancore |
| 2026-10-07 | 6.2a | rotate shared ranges | 3 seed, GPS semplice: base p95 0,77 -> 0,57 m, ospiti p95 1,13 -> 0,93 m, FA pre-attacco 0,034% -> 0%; post-attacco identico (<= 7 vicini). `nearest` = 4.3 byte per byte |
| 2026-10-07 | 6.2 | model anchor gps uncertainty | Test chi-quadro calibrato su 3 seed (0,10% oltre il 99,9%); receiver, oldest e consider di default |
| 2026-10-11 | 7.1 | add external gnss spoofer model | Tutti vedono la stessa posizione falsa; PPS = TDoA verso lo spoofer; simulatore identico al 6.2 |
| 2026-10-14 | 7.2 | add external spoofer to the simulator | Cattura totale vista nel simulatore; prima localizzazione dai PPS (ideale) |
| 2026-10-14 | 8.1 | detect swarm capture | 3 seed: 20/20 in 0,95 s sotto spoofer, 0% false catture con il nodo 0 bugiardo |
| 2026-10-14 | 8.2a | capture consensus, stop gps anchors | Consenso 20/20 in 1,05 s; niente piu' caos (build diverse identiche) |
| 2026-10-14 | 8.2b | relative uwb localization under capture | Forma 0,22 m sotto cattura (8.2a: 645 m); il riferimento scivola (340 m, 125 gradi) |
