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

# confronto con il passo precedente
python3 tools/compare.py runs/<prima>/summary.json runs/<dopo>/summary.json
```

Con la build nella cartella `scratch/` di ns-3, l'eseguibile si trova con:
`find build -type f -executable -name "*distry_mlat26v4*"` (dalla radice di ns-3).

Proprietà verificata in Fase 0: **stesso seed ⇒ output identico bit per bit.**

Attenzione: con `--netanim=1` il CSV differisce a livello di arrotondamento numerico (1 riga su
608.593 nel seed 1, colonna `mahal`, 5a cifra di un valore ~1e-8; metriche identiche). Per gli
esperimenti usare sempre `--netanim=0` (lo fa gia' `run_seeds.sh`).

Riferimento corrente: `reference/step2.1_seed1.json` (1 seed) e `reference/step2.1_seed1-3.json` (3 seed).
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
- [ ] **Fase 1 – Bug che non cambiano il modello** (le metriche devono restare identiche, salvo dove previsto)
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
  - [ ] `setSpeed`, `setScenary`, `t_join`, `t_leave` passati ma ignorati da `Trajectories.cpp`
  - [ ] Avvelenamento delle ancore: il nodo che mente viene usato come ancora ⇒ 61 % di falsi allarmi
  - [ ] Mahalanobis calcolata solo sulla misura diretta, il cui residuo è assorbito dal bias di clock
  - [ ] Allarme collettivo mai usato (nessuna esclusione del nodo sospetto)
  - [ ] Slot TDMA scelto dagli ospiti ma non usato per trasmettere; JOINING → ACTIVE senza verifica.
        Visto in 1.2: i droni 12, 13, 17, 18 scelgono tutti lo slot 9 e vengono tutti confermati
        (il controllo collisioni confronta l'ID del mittente con lo slot, non lo slot con lo slot)
  - [ ] Uscita dal geofence che scatta due volte per ogni ospite (dopo il goodbye `m_pendingLeave`
        torna false e lo stato resta ACTIVE): innocuo oggi, ma logica sbagliata
  - [ ] Da indagare: seed 2, ospite 19 con 7.089 falsi allarmi prima dell'attacco (seed 1: 586)
  - [x] 2.1 Ospiti fuori dal geofence non elaborano piu' i pacchetti (prima stimavano e scrivevano
        nel CSV da t=0). Le loro righe abbassavano l'errore EKF dei base (p95 0,67 m contro 1,25 m):
        ora p95 1,31 m, piu' onesto
- [ ] **Fase 3 – Radio realistica** (portata UWB, perdita pacchetti vs distanza, airtime) al posto del Wi-Fi a 30 dBm
- [ ] **Fase 4 – Clock realistici** (offset, deriva ppm, jitter) e ranging dai timestamp dei pacchetti
- [ ] **Fase 5 – Errore UWB realistico** (LOS/NLOS correlato nello spazio e nel tempo)
- [ ] **Fase 6 – Ricevitore GNSS** (satelliti, pseudodistanze, minimi quadrati per posizione e clock bias)
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
