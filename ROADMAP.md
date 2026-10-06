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

Riferimento corrente: `reference/step4.3_seed1.json` (1 seed) e `reference/step4.3_seed1-3.json` (3 seed).

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
  - [ ] Valutare un test chi-quadro sulla discrepanza (stima - GPS) con la covarianza del filtro, al posto
        della soglia max(2 m, 3,5 x pos_std)
  - [ ] Ogni drone condivide solo i 10 range piu' vicini: con gli ospiti dentro, alcuni range tra
        droni base non vengono piu' condivisi (esperimento E1: senza i range degli ospiti p95 8 m)
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
  - [~] 6.2 (in corso) Test chi-quadro implementato e attivo solo con `--gnss=receiver` (default invariato):
        d2 = e' S^-1 e, S = P filtro + 2 x accuratezza GPS individuale (trasmessa nel pacchetto, 4 byte);
        colonna `gps_chi2` nel CSV; soglia `--chi2Thr` (default 16,27). Seed 1: media d2 3,28 (calibrata),
        ma 1,3% oltre soglia (atteso 0,1%), FA collettivi pre-attacco 1,7% (prima 25%), latenza 4,6 s.
        CAUSA delle code: il multipath CORRELATO (10 s) delle ancore. Esperimenti seed 1: solo errore comune
        -> stima base p95 0,67 m, FA 0,05%; senza multipath -> 1,70 m, 0,05%; solo errori individuali ->
        5,42 m, 1,7%. L'effetto si amplifica con gli ospiti nello sciame (meccanismo da dimostrare).
        Prossimo: incertezza GPS di ogni ancora nella varianza di ciascuna misura (R_i) e gestione
        dell'errore lento; poi `receiver` di default.
  - [ ] (vecchia nota 6.2) Con `--gnss=receiver` i falsi allarmi esplodono (collettivi pre-attacco 10,3% +-13, seed 1 25%):
        la soglia max(2 m, 3,5 pos_std) e' tarata su un GPS con errore relativo ~0,3 m; quello realistico
        e' ~0,8 m (code 2-4 m), ~75% dei sospetti dominati dalla quota. Serve un test chi-quadro che usi
        covarianza del filtro + accuratezza dichiarata dal GPS (orizz./vert.), poi `receiver` di default
  - [ ] 6.3 Con il GPS realistico lo sciame si localizza nel riferimento del GPS (spostato dell'errore
        comune): l'errore rispetto alla posizione vera non e' piu' la metrica giusta per la localizzazione
- [ ] **Fase 7 – Spoofer esterno** (posizione, attivazione, cattura dei droni)
- [ ] **Fase 8 – Rilevamento dello spoofer esterno** con il consenso esistente
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
