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

# confronto con il passo precedente
python3 tools/compare.py runs/<prima>/summary.json runs/<dopo>/summary.json
```

Con la build nella cartella `scratch/` di ns-3, l'eseguibile si trova con:
`find build -type f -executable -name "*distry_mlat26v4*"` (dalla radice di ns-3).

Proprietà verificata in Fase 0: **stesso seed ⇒ output identico bit per bit.**

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
  - [ ] Salvare nel CSV anche il sospetto individuale (`myVote`), oggi non registrato
  - [ ] `m_isGuest` non inizializzato nel costruttore; `m_voteBitmask` inutilizzato e non inizializzato
  - [ ] `setSpeed`, `setScenary`, `t_join`, `t_leave` passati ma ignorati da `Trajectories.cpp`
  - [ ] Colori NetAnim: `i <= setnDrones` colora di blu anche il primo ospite
  - [ ] L'header dei range condivisi cresce con l'ID massimo invece che con il numero di range (10)
  - [ ] Codice morto (`PrintTerminalDashboard`, `GetVoteBitmask`), stampe di debug a istanti fissi
  - [ ] `viwer.py`: terminatori di riga Windows (CRLF); geofence disegnato in 3D ma calcolato in 2D
- [ ] **Fase 2 – Errori logici che falsano i risultati**
  - [ ] Avvelenamento delle ancore: il nodo che mente viene usato come ancora ⇒ 61 % di falsi allarmi
  - [ ] Mahalanobis calcolata solo sulla misura diretta, il cui residuo è assorbito dal bias di clock
  - [ ] Allarme collettivo mai usato (nessuna esclusione del nodo sospetto)
  - [ ] Slot TDMA scelto dagli ospiti ma non usato per trasmettere; JOINING → ACTIVE senza verifica
  - [ ] Da indagare: seed 2, ospite 19 con 7.089 falsi allarmi prima dell'attacco (seed 1: 586)
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
