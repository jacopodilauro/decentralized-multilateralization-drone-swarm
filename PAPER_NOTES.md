# PAPER_NOTES – note private per il paper (ACM MobiSys 2027)

> File ignorato da git (`.gitignore`). Contiene solo numeri verificati nel simulatore, con la
> configurazione in cui sono stati ottenuti. Aggiornato al 20 ottobre 2026 (Fase 3 su GitHub; direzione con UWB a S = 2: causa e correzione).

## 0. Scadenze e regole (CFP ufficiale MobiSys 2027, verificata il 21/10/2026: sigmobile.org/mobisys/2027/call_for_papers)

- Registrazione abstract: **27 novembre 2026 23:59 AoE** (in Italia 28 novembre alle 12:59). Titolo e abstract di almeno
  100 parole, significativi e vicini alla versione finale (abstract tipo "TBD" = rifiuto senza revisione)
- Paper: **4 dicembre 2026 23:59 AoE** (in Italia 5 dicembre alle 12:59). **Nessuna proroga**
- Early rejection 26/1/2027, rebuttal 8-12/2/2027 (500 parole), decisione 6/3/2027, camera-ready 14/4/2027
- Conferenza: giugno 2027, Ho Chi Minh City (presenza obbligatoria di un autore, quota non studente)
- Formato: **12 pagine** di contenuto principale (figure e tabelle comprese), appendice e bibliografia senza limite;
  `\documentclass[sigconf,10pt]{acmart}`, PDF. Submission su mobisys27.hotcrp.com
- **Revisione single-blind** (dal 2026): nomi e affiliazioni degli autori NEL paper. Il repository pubblico non e' un
  problema di anonimato (resta solo una questione di riservatezza delle note prima della sottomissione)
- **LLM:** il testo generato da LLM (ChatGPT, Claude, Gemini) va dichiarato chiaramente nel paper; non vale per
  revisione/correzione di testo scritto dagli autori. Vietato testo nascosto per influenzare revisori LLM
- La CFP 2027 enfatizza implementazioni funzionanti e riproducibili e valutazione pratica rigorosa: un paper solo
  simulato parte svantaggiato (misure UWB reali con DW3000, anche statiche, aiuterebbero molto). Esiste un track
  **Emerging Ideas** (titolo che inizia con "[Emerging Ideas]"): premia originalita' e idee non ancora del tutto
  validate. Scelta del track da discutere con il relatore
- Artifact evaluation opzionale dopo l'accettazione (il simulatore pubblico e' un punto a favore)

## 1. Obiettivo del lavoro

Uno sciame di droni senza infrastruttura che, usando solo le proprie radio UWB:
1. si localizza in modo cooperativo (distanze tra droni + multilaterazione distribuita),
2. **rileva** lo spoofing GNSS confrontando la posizione stimata con quella dichiarata dal GPS,
3. **localizza lo spoofer esterno** sfruttando il suo stesso segnale (TDoA sui clock bias GNSS),
4. gestisce ingressi e uscite di droni (membership dinamica) senza un coordinatore.

## 2. Architettura (stato attuale + pianificato)

| Blocco | Stato | Descrizione |
|---|---|---|
| MAC | fatto | TDMA statico: uno slot per ogni possibile membro, slot = ID, frame = N slot da 5 ms |
| Ranging | fatto | **DS-TWR ricavato dai broadcast TDMA** (nessun messaggio in piu'): ogni pacchetto porta timestamp di trasmissione e istanti di ricezione dai vicini (8 byte per vicino) |
| Localizzazione | fatto | EKF per ogni drone osservato (pos, vel, bias), distanze dirette + condivise da altri droni (10 per pacchetto, **a rotazione**), **compensazione del ritardo**, **R_i con l'incertezza GPS dell'ancora** |
| Rilevamento | fatto | test chi-quadro d2 = e' S^-1 e (99,9%), S = P + covarianza GPS del mittente + covarianza "consider" del bias da ancore; isteresi 10 campioni, voto a quorum ceil(2/3) |
| Difesa | fatto | i droni sotto allarme collettivo non sono usati come ancore |
| Membership | fatto | geofence, ascolto passivo, join, leave volontario e eviction a quorum |
| Ricevitore GNSS | fatto (6.1–6.2, default) | pseudodistanze e clock bias; covarianza piena dell'errore individuale nel pacchetto (9 byte) |
| Spoofer esterno | fatto (7.1–7.2) | singola antenna, cattura di tutti i droni, `--spoof=1`, log `gnss_log.csv` |
| Rilevamento cattura | fatto (8.1–8.2a) | coerenza GPS/UWB su tutte le coppie di altri droni (< 25% coerenti per 1 s) + consenso 2/3 (bit nel pacchetto); sotto consenso niente caccia al bugiardo e niente ancore GPS |
| Localizzazione spoofer | da fare (Fase 9) | TDoA sulle differenze di clock bias, corrette con gli offset UWB tra droni |

Nel paper scrivere "range-based cooperative multilateration" (in aviazione MLAT = TDoA, evitare ambiguita').
Il TDoA compare davvero solo nella localizzazione dello spoofer.

## 3. Contributi candidati (da rifinire dopo la Fase 9)

1. Localizzazione dello spoofer GNSS esterno da parte di uno sciame **completamente catturato e senza
   infrastruttura**: differenze di PPS (il PPS sotto spoofing scatta in ritardo della distanza dallo spoofer,
   l'orologio del ricevitore si cancella) sincronizzate con gli orologi UWB, geometria dei ricevitori dal ranging
   UWB. **Posizionamento (ricerca bibliografica del 14/10, 3 ricerche, NON sistematica):** la localizzazione TDoA
   dello spoofer da piu' ricevitori GNSS esiste (Broumandan 2015; Gamba 2016; Bhamidipati & Gao, IEEE TAES 2019;
   QSSL, NAVIGATION 2022; Stanford, ION GNSS+ 2025), ma richiede ricevitori fermi in posizioni note, sincronizzazione
   dedicata (orologi atomici/SDR in Bhamidipati & Gao) oppure segnali autentici ancora visibili (QSSL risolve posizione
   e tempo dai segnali autentici; Stanford richiede almeno un satellite falso e uno autentico). Nel nostro caso i
   ricevitori sono in volo, completamente catturati, a basso costo e non conoscono la propria posizione: lo sciame
   fornisce geometria (UWB) e sincronizzazione (DS-TWR + PPS). Nella letteratura sugli sciami il ranging UWB serve a
   RILEVARE lo spoofing (arXiv 2301.12766, Frontiers 2026, arXiv 2608.06885), non a localizzare lo spoofer.
   **Prima di scrivere "first": ricerca sistematica (Scholar, IEEE Xplore, ACM DL) e parere del relatore.**
   Caveat: la geometria UWB e' relativa (senza orientamento): lo spoofer si localizza rispetto allo sciame; per le
   coordinate geografiche serve un riferimento (ultima posizione buona, bussola, IMU). Da trattare nella Fase 9.
2. Analisi del ritardo delle misure condivise in un TDMA broadcast: errore sistematico v·Δt che con
   droni veloci domina l'errore UWB e cresce con la dimensione dello sciame (frame = N slot); compensazione
   con modello di misura ritardata nell'EKF.
3. Difesa dall'avvelenamento delle ancore con il consenso gia' presente.
4. Sistema integrato (ranging, localizzazione, rilevamento, consenso, membership) in un unico MAC.

Nota onesta: il DS-TWR e la gestione delle misure ritardate (out-of-sequence measurements) sono tecniche
note; la novita' sta nell'integrazione e nella localizzazione dello spoofer.

## 4. Risultati verificati (simulatore)

Configurazione di default: 8 droni base (ID 0–7, nodo 0 fermo al centro, 1–7 su sfera r=10 m in
rotazione) + 12 ospiti (ID 8–19, orbita r=15 m a 7,5 m/s tra ~30 e ~175 s), 240 s, slot 5 ms, frame 0,1 s,
attacco sul nodo 0 a 200 s (GPS spostato di 15 m in y con rampa di 10 s). Metriche su seed 1–3.

### 4.1 Prima e dopo (codice iniziale v0-baseline → passo 4.3)

| Metrica | v0 | 4.3 |
|---|---|---|
| Falsi allarmi collettivi sui droni onesti dopo l'attacco | 61,0% | **0%** |
| Allarmi collettivi falsi prima dell'attacco | 1,36% | **0,034%** |
| Errore di stima droni base, mediana / p95 | 0,45 / 1,26 m | **0,27 / 0,77 m** |
| Latenza di rilevamento (allarme collettivo) | 2,40 s | 2,34 s |
| Osservatori che rilevano il nodo 0 | 7/7 | 7/7 |
| Orologi | ±1 ns, fissi (irrealistici) | offset casuale su 40 bit, skew ±20 ppm, jitter 10 ps |

### 4.2 Ritardo delle misure condivise (passi 2.2–2.3)

- Eta' media di un range condiviso al momento dell'uso: ~0,12 s. Velocita': base 1,1 m/s, ospiti 7,5 m/s.
- Con ospiti nello sciame l'errore dei base **raddoppiava** (p95 0,78 → 1,40–1,51 m, seed 1).
- Esperimenti oracolo (seed 1, p95 base nei periodi con ospiti): range corretti per il ritardo → 0,76–0,85 m
  (problema risolto); ancore con posizione vera → nessun effetto (ipotesi scartata); scartare range
  vecchi → solo parziale. Confermato su seed 2 e 3.
- Compensazione (eta' nel pacchetto, h = |p − v·Δ − a(t_m)|): p95 base con ospiti 1,40 → 0,84 m.
  Test isolato: bersaglio a 7,5 m/s con range vecchi 0,12 s, errore 0,95 m senza compensazione,
  0,28 m con compensazione (0,27 m senza ritardo).
- Effetto collaterale: con q_acc = 0,5 il filtro non segue le curve degli ospiti (3,75 m/s²) e i falsi
  allarmi raddoppiavano. Sweep q_acc 0,5–5: salto netto tra 0,5 e 1–2, poi andamento graduale → q_acc = 2.

### 4.3 Avvelenamento delle ancore (passo 2.4)

- Il drone che mente ha range veri ma posizione dichiarata falsa: usato come ancora trascina le stime
  dei droni onesti → 60% di falsi allarmi su onesti.
- Oracolo (mai il nodo 0 come ancora) = 0%; esclusione delle ancore sotto allarme collettivo = 0%
  (stesso risultato con informazioni reali); gating sui residui da solo = 14,8%.
- Errore sui droni onesti dopo l'attacco: 3,38 / 10,8 m → 0,50 / 0,99 m (3 seed).

### 4.4 Altri risultati

- Join TDMA originale: slot scelto mai usato, collisioni finte, mappe incoerenti → 3.833 voti attribuiti
  al drone sbagliato e 219.056 voci incoerenti (seed 1). Ora TDMA statico: 0 e 0.
- Mahalanobis della misura diretta: causava ~70% dei sospetti falsi e 0% dei rilevamenti (residuo
  assorbito dal bias di clock). Tolta dalla decisione: sospetti istantanei 1,31% → 0,40%.
- Offset degli orologi (modello vecchio): correlazione offset/errore di stima **−0,95** su 36 ospiti
  (3 seed); offset negativi 0,87 m contro positivi 0,43 m di errore mediano. Oracolo senza offset
  (seed 2): falsi allarmi pre-attacco 0%, errore base mediano 0,39 → 0,25 m.

### 4.5 DS-TWR (Fase 4)

- Test isolato, droni fermi, offset casuali e skew ±20 ppm: **1,2 mm RMS**. ToA a una via con gli stessi
  orologi: errore minimo **468 km** (inutilizzabile). Jitter 10 ps: 2,8 mm RMS. Azzeramento del
  contatore a 40 bit gestito.
- **In movimento il DS-TWR restituisce la distanza all'istante del messaggio centrale**: errore 2–3 mm
  anche a 15 m/s; riferito a meta' intervallo sbaglierebbe fino a 33 cm. Al primo ordine
  ToF ≈ ½τ₂ + Da/(2(Da+Db))·τ₁ + Db/(2(Da+Db))·τ₃.
- Costo: eta' alla condivisione = 1 frame esatto (prima mezzo frame in media), +0,05 s.
- Nel simulatore, contro la verita' (seed 1, 576.545 scambi, tutti validi): LOS errore medio 0,03 cm,
  dev. std 7,1 cm; NLOS bias medio 55,6 cm. Serve il bit LOS della ricezione dell'altro drone: senza,
  in LOS dev. std 19 cm e picchi di 2,4 m.
- Pacchetto con 20 droni: 132 → 291 byte.
- Su 3 seed rispetto al 2.6: errore base 0,39/1,01 → 0,27/0,77 m; ospiti 0,61/1,42 → 0,62/1,13 m;
  falsi allarmi collettivi pre-attacco 0,19% → 0,034%; rilevamento invariato. La variabilita' tra seed
  dell'errore base scende da ±0,032 a ±0,0013 m.
- Alternativa valutata e scartata: broadcast a una via + stima di offset e deriva di ogni coppia dalle due
  direzioni (2–4 mm, anche in orbita con finestra 2–5 frame). Scartata perche' produce misure gia'
  filtrate (errori correlati) da dare all'EKF di posizione (filtri in cascata → troppa fiducia), a parita'
  di costo di rete. Gli offset tra orologi si possono ricavare dagli stessi timestamp DS-TWR.

### 4.6 Ricevitore GNSS realistico (Fase 6.1)

- Modello: errore comune per satellite (uguale per droni vicini, 1,5 m, Gauss-Markov 600 s) + multipath
  individuale (0,4 m, 10 s) + rumore di codice (0,2 m); orologio del ricevitore +-1 ms, +-0,5 ppm.
  **Parametri = ipotesi**, da giustificare con fonti (schede tecniche ricevitori consumer, ~2-2,5 m CEP)
  e da variare in un'analisi di sensibilita'.
- Test isolato: errore assoluto orizzontale 2,6 m (p95 4,7), relativo tra droni a 20 m 0,70 m (p95 1,6);
  clock bias stimato entro 1,3 m (mediana). Nel simulatore: assoluto 2,4 m, relativo 0,78 m (p95 1,8).
- **Risultato importante**: con il GPS realistico la soglia di sospetto attuale non regge. Allarmi
  collettivi falsi pre-attacco 0,034% -> 10,3% (+-13 tra seed; seed 1 25%, seed 3 0,6%, dipende dalla
  geometria dei satelliti); ~75% dei sospetti falsi dominati dalla quota. Il vecchio GPS (rumore bianco
  0,2 m) nascondeva il problema. Messaggio per il paper: la soglia deve includere l'incertezza del GPS
  (test chi-quadro con covarianza del filtro + accuratezza dichiarata dal ricevitore, orizz./vert.).
- Lo sciame si localizza nel riferimento del GPS (spostato dell'errore comune ~2 m): per la localizzazione
  va misurato l'errore relativo, non quello assoluto.

### 4.7 Test chi-quadro calibrato (6.2, chiuso)

Default: `--gnss=receiver --sharePolicy=oldest --anchorModel=consider`. Riferimento `reference/step6.2_seed1-3.json`.
Il 4.3 si riottiene con `--gnss=simple --sharePolicy=nearest` (CSV identico byte per byte, verificato).

**Tre cause delle code, trovate con oracoli e test isolati:**

1. **Fame di range dei droni periferici** (bug preesistente, anche col GPS semplice). Condividere i 10 range piu'
   vicini lascia i droni periferici senza range condivisi (seed 1, nodo 3: nessun condivisore a ~77–90 s e
   ~140–152 s, verificato sulla verita' a terra): < 4 misure, EKF solo in predizione, pos_std ~48 m, errore fino a
   32–90 m. Nascosto dal p95 (~3,6% delle righe). **Implicazione di sicurezza: con pos_std a 40 m il test e' cieco,
   un attaccante periferico non sarebbe rilevato.** Era la vera causa dell'"amplificazione con gli ospiti" (non la
   quota). Correzione: rotazione (i K range condivisi meno di recente), stessa dimensione del pacchetto.
   Seed 1, GPS semplice: base p99 5,54 -> 0,74 m (oracolo con tutti i range 0,68), max 31,7 -> 2,4 m.
   3 seed, GPS semplice: base p95 0,77 -> 0,57 m, ospiti 1,13 -> 0,93 m, FA pre 0,034% -> 0%; post-attacco
   identico (con 8 droni "piu' vicini" e rotazione coincidono).
2. **Errore GPS delle ancore lento trattato come bianco.** Test NEES (bersaglio che si muove col modello del
   filtro, 1000 prove, q_acc 2; R_i = sigma2_uwb + u'Sigma u): ancore esatte d2 2,91; ancore bianche **3,00**
   (filtro consistente); ancore Gauss-Markov 10 s **10,0 e 16,4% oltre il 99,9%** (q_acc 0,5: 13,4 e 29%);
   aggiungendo la covarianza consider G D G' (G = minimi quadrati pesati sulla geometria corrente,
   D = diag(u'Sigma u)): 2,43 e 0%. Messaggio: piu' memoria ha il filtro, piu' e' troppo sicuro di se'.
   Nota metodologica: su bersaglio FERMO P risulta conservativa (~0,8x) anche con ancore esatte (mismatch del
   modello di moto): la consistenza si verifica solo con moto coerente col modello.
   Varianze UWB onesti dal modello di canale: LOS 0,01 m2, NLOS 0,36 m2 (il valore storico 1,5 m2 gonfiava P ~3x).
   Il bit NLOS dei range condivisi prima non era trasmesso: ora nel bit alto del campo range (0 byte).
3. **Anisotropia del GPS.** Il modello GNSS e' onesto (std normalizzata 0,96–1,02), ma hAcc unico perde la forma
   dell'ellisse (seed 1: sigma x 0,62, y 0,34 m, hAcc 0,47). 30 costellazioni: diag(h,h,v) -> media d2 3,01 ma
   0,44% oltre soglia; covarianza piena -> 3,04 e 0,08%. Pacchetto: sigma E/N/U + 3 correlazioni, 9 byte (+5).
   NMEA GST fornisce l'ellisse orizzontale; **per la covarianza completa (u-blox UBX-NAV-COV) verificare la
   documentazione prima di citarla.**

**Risultati, 3 seed, receiver (6.2 iniziale -> 6.2 finale):**

| Metrica | 6.2 iniziale (nearest+fixed) | oldest+fixed | oldest+r | **oldest+consider** |
|---|---|---|---|---|
| d2 medio (atteso 3) | 2,67 | 2,21 | 3,32 | **2,76 ±0,18** |
| oltre il 99,9% (atteso 0,1%) | 0,54% | 0,23% | 0,35% | **0,10 ±0,06%** |
| errore relativo base / ospiti p95 | 2,69 / 1,96 m | 1,61 / 1,52 | 1,20 / 1,16 | **1,20 / 1,16 m** |
| FA individuali / collettivi pre | 0,36 / 0,60% | 0,12 / 0% | 0,09 / 0,06% | **0 / 0%** |
| FA collettivi onesti post | 0,39% | 0,39% | 3,09% | 0,80 ±1,39% |
| latenza (7/7) | 3,57 s | 3,57 s | 2,70 s | **3,04 ±0,75 s** |

- `r` da sola migliora la stima ma rende il test troppo sicuro: serve consider (ablazione utile per il paper).
- FA onesti post 0,80%: solo seed 1 (2,41%). L'oracolo senza attacco ne riproduce 2,18 punti (nodi 6 e 7 a 210 s):
  escursioni REALI del GPS del mittente (nelle righe in coda la stima dista 0,9–1,4 m dal riferimento, il GPS del
  mittente 4,3–4,8 m). Tutti gli osservatori le vedono insieme: **il quorum protegge dagli errori lato osservatore,
  non da un'escursione del GPS del mittente**. Nel paper: classificarle come allarmi di integrita' GNSS, separate dai
  falsi allarmi rispetto all'attacco. Residuo legato all'attacco ~0,2% (nodo 2, 200–205 s): ipotesi avvelenamento
  delle ancore prima del rilevamento, **da verificare**.
- Con multipath tau = 10 s, 20 droni e 240 s ci sono ~480 campioni indipendenti: la frazione oltre soglia di un
  singolo seed ha varianza alta (seed 1: 0,17%). Per le ROC servono >= 10 seed.

### 4.8 Spoofer esterno: modello e primo sguardo alla localizzazione (7.1, test isolato)

- Modello: rho_ij = |x_f - S_j| + c*beta + |p_i - s| + c*b_i + m_i + n_ij (m_i multipath del collegamento
  spoofer -> drone, IPOTESI 0,5 m / 10 s). Ai minimi quadrati tutti i droni vedono x_f (Tippenhauer, CCS 2011)
  e il clock bias contiene beta + |p_i - s|/c.
- **Osservabile proposto: il PPS.** Il ricevitore emette il PPS quando crede che inizi il secondo; sotto spoofing
  avviene all'istante vero N + beta + |p_i - s|/c: **l'orologio del ricevitore si cancella**. Ogni drone marca il
  PPS con l'orologio UWB; con gli offset UWB tra droni (dai timestamp DS-TWR) le differenze sono TDoA.
  Da verificare con le fonti: jitter e quantizzazione del PPS dei ricevitori consumer (qErr u-blox), e se il
  ricevitore sposta il PPS subito o con un filtro sull'orologio dopo la cattura.
- Test [7] (8 droni, raggio 10 m, antenna a terra a 500 m): scarto max dalla posizione falsa 1,23 m; differenze
  di PPS contro differenze di distanza vere RMS 1,00 m; senza rumore errore 0.
- Test [8], informativo (60 s di PPS a 1 Hz, offset UWB ideali, droni fermi, 200 prove, mediana / p95):

| formazione | jitter PPS | azimut [gradi] | elevazione [gradi] | distanza [m] |
|---|---|---|---|---|
| raggio 10 m | 0 ns | 0,71 / 2,22 | 0,66 / 1,95 | non determinabile |
| raggio 10 m | 5 ns | 0,91 / 2,37 | 0,88 / 2,31 | non determinabile |
| raggio 10 m | 20 ns | 2,26 / 5,99 | 2,24 / 5,71 | non determinabile |
| raggio 50 m | 0 ns | 0,18 / 0,46 | 0,13 / 0,40 | 47 / 162 |
| raggio 50 m | 5 ns | 0,19 / 0,56 | 0,17 / 0,47 | 61 / 217 |
| raggio 50 m | 20 ns | 0,49 / 1,17 | 0,46 / 1,21 | 156 / 1.928 |

  Con base corta la direzione e' buona e la distanza no (come previsto); con 50 m la distanza diventa usabile.
  Leve per la Fase 9: formazione piu' aperta, rotazione dello sciame nel tempo (apertura sintetica), integrazione
  piu' lunga, EKF sulla posizione dello spoofer.

### 4.9 Spoofer nel simulatore (7.2, seed 1)

Comando: `--spoof=1 --spoofTime=100` (antenna a 500 m, azimut -37 gradi, quota 2 m; posizione falsa = centro + 15 m
in y). Analisi: `python3 tools/spoofer_view.py runs/<etichetta>/seed1 [--jitter-ns 20]`.

- Cattura: 20/20 droni in 0,1 s; dispersione delle posizioni GPS tra droni 0,66 m (vere 13,4 m): tutti vedono lo
  stesso punto falso (Tippenhauer confermato nel simulatore).
- Sistema attuale: allarmi collettivi 0 -> 84,5%, chi2 mediano 2,4 -> 162, pos_std 0,35 -> 2,37 m. Segnala tutti
  come bugiardi e la stima peggiora: e' progettato per pochi droni che mentono. **Fase 8: riconoscere la cattura
  totale (incoerenza UWB/GPS a livello di sciame) e passare a una localizzazione solo relativa (UWB).**
- Localizzazione dai PPS a 1 Hz (geometria e sincronizzazione IDEALI):

| dopo | apertura | azimut | elevazione | distanza (vera 500 m) |
|---|---|---|---|---|
| 5 s | 32 m | -0,23 | +0,36 | -246 m |
| 10 s | 32 m | +0,58 | +0,20 | -185 m |
| 20 s | 36 m | +0,25 | -0,13 | -83 m |
| 60 s | 58 m | +0,31 | +0,26 | +320 m |
| 120 s | 88 m | +0,13 | +0,21 | +42 m |

  Jitter 20 ns: direzione instabile nei primi 10 s, poi 0,6–2,5 gradi; a 120 s 0,76 gradi e +75 m.
  **Il moto dello sciame allarga l'apertura nel tempo (apertura sintetica)**: da sfruttare nella Fase 9 (EKF
  sullo spoofer). La distanza oscilla nei primi 60 s (multipath correlato 10 s del collegamento, ipotesi).

- **Nota di metodo (7.2):** sotto cattura il sistema attuale e' caotico (allarmi ed esclusioni di ancore si
  inseguono): tra la macchina di Jacopo e quella di Claude allarmi 83,3% contro 84,5%, chi2 169 contro 162, mentre
  cattura e localizzazione sono identiche. Verificato: ricompilando con contrazione FMA si ottengono esattamente i
  numeri di Jacopo; la regressione senza spoofer resta identica (1e-10). Regola: scenari con spoofer confrontati
  con tolleranza e su piu' seed; figure finali sulla macchina di Jacopo.

### 4.10 Rilevamento della cattura dello sciame (8.1)

- Test: ogni drone, una volta per frame, valuta le coppie di ALTRI droni (k, j): distanza tra i GPS dichiarati
  (riportati all'istante del range con la velocita' dei GPS dichiarati su ~1 s) contro range UWB condiviso.
  Coerente se |d_GPS - r| <= 4 sigma, sigma2 = u'(Sigma_k + Sigma_j)u + 0,36 + (2 m/s)^2 (dt_k^2 + dt_j^2).
  Cattura se meno del 25% delle coppie e' coerente (>= 6 coppie) per 10 frame (1 s).
- Perche' le coppie degli altri: con le sole proprie coppie un drone non distingue "sono ingannato solo io" da
  "siamo ingannati tutti". Con k bugiardi su N restano coerenti C(N-k, 2) coppie: regge fino a quasi meta' sciame.
- Risultati, 3 seed:
  - nodo 0 bugiardo (attacco interno): 0% false catture prima e dopo; coppie coerenti 100% prima, 81% dopo;
    minimo senza attacco 98,8%;
  - spoofer esterno a 100 s (20 droni): 20/20 rilevano in 0,95 s (= isteresi: scatta al primo frame), 100% del
    tempo in cattura, coppie coerenti 1-8% (restano compatibili solo le coppie a meno di ~5 m);
  - spoofer a 200 s (8 droni, seed 1): 8/8 in 1,0 s.
- **Principio di progetto (da scrivere nel paper):** il test di cattura non deve dipendere da stati stimati con le
  ancore GPS. La prima versione usava le velocita' degli EKF: sotto cattura divergono (30-80 m/s) e rendevano
  "coerenti" a caso il 25-33% delle coppie (un drone usciva dalla cattura). Con la velocita' dai GPS dichiarati il
  problema sparisce.
- Localizzazione dai PPS (ideale) sugli stessi 3 seed, dopo 120 s: distanza +42 / +3 / -5 m su 500, direzione
  entro 0,35 gradi; nei primi 10 s la distanza e' instabile (seed 3 non determinata).
- Limiti: sciame molto compatto (droni a pochi metri) -> test debole; uno spoofer che ingannasse ogni drone con una
  posizione diversa e coerente (piu' antenne, offset rigido) non e' rilevabile da misure solo relative (sez. 5).

### 4.11 Consenso sulla cattura (8.2a, seed 1)

- Ogni drone annuncia il proprio stato di cattura (1 bit, 0 byte in piu'); sciame catturato se >= 2/3 dei droni
  attivi lo annunciano. Sotto consenso: nessun sospetto "bugiardo" e nessun aggiornamento dei filtri con ancore GPS.
- Spoofer a 100 s: consenso di 20/20 droni in 1,05 s, 100% del tempo, mai interrotto. Allarmi "bugiardo" sugli
  onesti 83% -> 1,4% (solo il secondo prima del consenso).
- **Fine del caos:** build normale e build con contrazione FMA danno ora metriche identiche anche sotto spoofer
  (prima allarmi 83,3% contro 84,5%). Messaggio: riconoscere la cattura rende il sistema stabile e riproducibile.
- Costo: senza ancore i filtri solo predicono (pos_std mediana 225 m). Serve l'8.2b (localizzazione relativa UWB).

### 4.12 Localizzazione relativa sotto cattura (8.2b, seed 1, spoofer a 100 s)

- Al consenso ogni drone riparte dall'istantanea dei suoi filtri di 1–2 s prima dell'attacco (presa solo senza
  sospetti); ancore = sue stime degli altri droni con la loro covarianza; un filtro su se stesso (dall'ultimo GPS
  buono, aggiornato con le distanze dirette) fa da ancora per le misure dirette.
- Metrica di **forma**: stime allineate alla verita' con la miglior rototraslazione (Kabsch), residuo RMS.

| | prima dell'attacco (GPS) | 8.2a (solo predizione) | 8.2b (relativa UWB) |
|---|---|---|---|
| forma mediana / p95 | 0,54 / 0,94 m | 645 / 1.475 m | **0,22 / 1,59 m** |
| pos_std mediana | – | 225 m | 0,18 m |
| deriva del riferimento: traslazione / rotazione | – | 515 m / 150 gradi | 340 m / 125 gradi |

- La forma e' mantenuta (meglio che con il GPS, i cui errori non ci sono piu'), ma **il riferimento scivola**:
  posizione e orientamento dell'insieme non sono osservabili da sole distanze; la velocita' comune dei filtri fa un
  random walk (~t^1,5). Per la Fase 9: la forma basta per localizzare lo spoofer RISPETTO allo sciame; per una
  direzione nel mondo serve un riferimento indipendente dal GPS -> 8.2c (velocita' da IMU/odometria, direzione da
  bussola, condivise nel pacchetto). Da verificare in letteratura: precisione tipica di velocita' IMU/VIO e bussola
  su droni piccoli, per i parametri del modello.
- Limite: i droni che entrano durante la cattura non vengono inizializzati.

### 4.13 Localizzazione dello spoofer con la geometria dello sciame (9.0, analisi dei log, seed 1)

`python3 tools/spoofer_locate.py runs/<etichetta>/seed1 --observer 1 --png figura.png` (run con `--spoof=1
--spoofTime=100`). PPS a 1 Hz, sincronizzazione IDEALE, spoofer a 500 m. Osservatore 1, mediane dopo 20 s:

| geometria | errore di direzione | errore di distanza |
|---|---|---|
| ideale (posizioni vere) | 0,18 gradi | 103 m |
| stimata dallo sciame (8.2b), senza 8.2c | 18,5 gradi | 1.857 m (inutilizzabile) |
| stimata dallo sciame, con 8.2c ideale | **0,88 gradi** | **129 m** |

- Diagnosi: il riferimento stimato ruota di ~3–5 gradi/s (picchi 60–90 gradi/s) e le misure di istanti diversi
  diventano incoerenti; la forma dei droni base e' buona (errore per drone 0,16–0,21 m mediana, p95 0,44–0,60 m).
  Osservatore 5: con 8.2c ideale 0,4–0,7 gradi, distanza -55...-97 m dopo 90–120 s. Gli ospiti (veloci, ai bordi)
  hanno una forma molto peggiore (osservatore 12: 5 m / 25 m).
- **Messaggio: con la propria geometria UWB lo sciame localizza lo spoofer a ~1 grado e ~10–25% in distanza, a
  patto di ancorare il riferimento (8.2c). Senza ancoraggio non funziona.**
- "8.2c ideale" = stime riportate nel mondo ogni secondo con la miglior rototraslazione verso la verita' (toglie
  solo rotazione e deriva, lascia gli errori di forma). Il vero 8.2c (IMU/bussola) dara' un risultato tra le due.

### 4.14 Ancoraggio del riferimento con i sensori di bordo (8.2c, seed 1, spoofer a 100 s)

- Modello (`ImuModel.h`): velocita' nord/est/su con rumore 0,2 m/s, bias 0,3 m/s (Gauss-Markov 30 s), scala 3%,
  bussola 3 gradi (Gauss-Markov 120 s). Fonti per l'optical flow: in letteratura errori di qualche decimo di m/s
  (es. 0,3-0,6 m/s medi su piccoli UAV simulati, JNTETI 2025) e 2-3% della velocita'; **bussola = ipotesi da
  verificare**. Sotto cattura ogni drone usa e annuncia (6 byte) la propria velocita' misurata.

| | senza 8.2c | con 8.2c |
|---|---|---|
| forma mediana / p95 | 0,22 / 1,58 m | 0,19 / 0,28 m |
| deriva del riferimento | 340 m, 125 gradi | 4,7 m, 8,3 gradi |
| **localizzazione REALE, direzione** (mediana dopo 20 s) | 86 gradi | **0,62 gradi** (oss. 1), 1,58 (oss. 5) |
| localizzazione reale, distanza | non determinata | non determinata |
| con "8.2c ideale" (solo errori di forma) | 0,88 gradi / 129 m | 0,23 gradi / 57 m |

- **Primo risultato senza aiuti ideali sulla geometria** (sincronizzazione ancora ideale): lo sciame catturato
  trova la direzione dello spoofer a ~1 grado con le sole proprie misure.
- Distanza: la deriva residua (pochi m e gradi in 140 s) cancella la curvatura del fronte d'onda (pochi cm a 500 m)
  accumulando tutte le misure. Finestre di 30-60 s: distanza determinata nel 73-88% dei casi, errore 160-300 m, ma
  direzione 2,6-4,4 gradi. Soluzione prevista: filtro sullo spoofer che stima anche la deriva (9.2).

### 4.15 Sincronizzazione reale: PPS e orologi UWB (9.1, seed 1, spoofer a 500 m)

- Ogni drone catturato genera il PPS a ogni secondo intero (istante vero = N + bias stimato - bias vero), con il
  dente di sega del ricevitore (uniforme +-10 ns) e 1 ns di rumore; lo marca con il proprio orologio UWB (offset,
  deriva +-20 ppm, jitter) e lo annuncia. Ogni osservatore stima la relazione tra il proprio orologio e quello del
  mittente (retta su ~2 s di coppie tx/rx corrette per il tempo di volo DS-TWR) e calcola la differenza di PPS.
- Fonti per il PPS: u-blox dichiara l'errore di quantizzazione del prossimo impulso (qErr in UBX-TIM-TP), quindi
  compensabile; utenti riportano dente di sega entro +-10 ns su NEO-M8N; LEA-M8F <= 20 ns a cielo aperto.
  **Ipotesi da verificare su hardware: sotto spoofing il PPS segue subito il tempo falso (nessun filtro
  sull'orologio del ricevitore).**
- Errore di sincronizzazione (misura - ideale): std 2,47 m; atteso dal solo dente di sega 2,45 m -> la relazione
  tra orologi UWB e' quasi perfetta, domina il PPS. Con qErr compensato: 0,50 m.
- Localizzazione, da 60 s in poi (mediane):

| | direzione oss. 1 | direzione oss. 5 | elevazione | distanza |
|---|---|---|---|---|
| geometria e sync ideali | 0,19 gradi | 0,23 gradi | 0,24-0,43 gradi | 77-96 m |
| geometria stimata + sync ideale | 0,35 | 0,42 | 1,06-1,41 | non determinata |
| **stimata + sync REALE (nessuna ipotesi ideale)** | **0,54** | **0,15** | 1,73-2,05 | non determinata |
| stimata + sync reale, qErr compensato | 0,23 | 0,12 | 1,29-1,40 | non determinata |

- **CORREZIONE (14/10, strada A): i valori sopra sono dei soli osservatori 1 e 5, tra i migliori. Su tutti gli 8
  osservatori base (S = 1, stima finale con tutte le misure) la direzione ha mediana 1,4 gradi e va da 0,31 a
  4,55 gradi (oss. 0: 4,55; 1: 0,66; 2: 2,33; 3: 2,14; 4: 0,38; 5: 0,31; 6: 2,78; 7: 0,39). Messaggio corretto: senza
  ipotesi ideali lo sciame trova la direzione con ~1-2 gradi in mediana, sotto 1 grado per meta' dei droni.**
  Da capire: perche' alcuni osservatori stimano peggio; fondere le stime degli osservatori.
- Aperti: elevazione (deriva verticale del riferimento) e distanza (9.2: filtro che stima anche la deriva).

### 4.16 Perche' la distanza non si ottiene con questo sciame (9.2, prototipo, seed 1)

- Prototipo (solo analisi, non consegnato): filtro di Kalman sulla posizione apparente dello spoofer nel riferimento
  dello sciame (random walk q = 0,1-4 m2/s, aggiornamento iterato, partenza a 1-2 km con 3 km di incertezza in
  distanza). Geometria ideale: distanza 54-200 m (come i minimi quadrati). Geometria REALE: converge stabilmente a
  ~-360 m (stima ~140 m invece di 500) oppure diverge; direzione 3-6 gradi (peggio dei minimi quadrati, 0,5).
- Causa (misurata): il **segnale di curvatura** (differenze di distanza vere meno quelle di un'onda piana dalla
  stessa direzione) vale 0,10 m RMS per secondo (p10-p90 0,08-0,14 m; apertura orizzontale mediana 28 m), MENO
  dell'errore di forma della geometria stimata (0,19 m), che e' lento e sistematico e non si media nel tempo.
  Ordine di grandezza: curvatura ~ (a/2)^2 / (2R): a 500 m, apertura 30 m -> 0,23 m; 90 m -> 2 m.
- **Messaggio per il paper: la direzione e' robusta (0,1-0,5 gradi senza ipotesi ideali); la distanza richiede che
  il segnale di curvatura superi l'errore di forma: aperture ampie (~3x l'attuale a 500 m) oppure triangolazione
  delle direzioni nel tempo mentre lo sciame trasla (parallasse: 100 m di spostamento a 500 m = ~11 gradi, molto
  piu' dell'errore di direzione), che richiede pero' un riferimento stabile per tutta la traslazione (oggi la
  rotazione residua dell'8.2c e' ~8 gradi in 140 s, piu' dell'errore della bussola modellato, 3 gradi).**
- Nel simulatore le traiettorie sono fisse (`setSpeed`/`setScenary` ignorati, sez. 6): per gli esperimenti sulla
  distanza servono un'opzione per il raggio della formazione e uno scenario con lo sciame in traslazione.

### 4.17 Strada A: distanza in funzione della scala della formazione (preliminare, seed 1)

- `--formationScale` S: distanze x S (sfera base diametro 20 S m, oscillazione verticale 12 S m, orbita ospiti
  diametro 30 S m, geofence), velocita' angolari / S (velocita' lineari invariate). S = 1 identico bit per bit.
- Analisi: `tools/aperture_sweep.py` (8 osservatori base, stima con tutte le differenze di PPS dal consenso alla fine;
  "determinata" = |errore| < 50% di R). Campagna: `tools/run_aperture_sweep.sh`.

| S | R | curvatura | errore di forma | distanza reale (mediana) | determinata | direzione reale (mediana) | distanza ideale |
|---|---|---|---|---|---|---|---|
| 1 | 500 m | 0,10 m | 0,19 m | indeterminata | 25% | 1,40 gradi (0,31-4,55) | 97 m |
| 2 | 500 m | 0,39 m | 0,20 m | 109 m (22%) | 87,5% | 0,44 gradi | 41 m |
| 3 | 500 m | 0,87 m | 0,19 m | 187 m (37%) | 75% | 1,69 gradi | 5,5 m |
| 4 | 500 m | 1,58 m | 0,22 m | 42 m (8%) | 100% | 1,49 gradi | 1,4 m |
| 2 | 1000 m | 0,20 m | 0,20 m | 425 m (42%) | 50% | 0,42 gradi (0,15-6,01) | 180 m |

- **Regola confermata: la distanza diventa misurabile quando la curvatura (~ (a/2)^2 / 2R) supera l'errore di forma
  (~0,2 m, indipendente dalla scala).** A R = 1000 m, S = 2 i due sono uguali e la distanza e' al limite.
- Un solo seed: le oscillazioni (S = 3 peggio di S = 2, direzione non monotona) sono rumore. Servono >= 5 seed.
- Apertura misurata (mediana per secondo) solo informativa: gli ospiti escono a ~167 s e la mediana salta tra i due
  regimi; le figure usano la scala S.

### 4.18 Strada A: campagna completa (PC di Jacopo, 60 run: S 1-4 x R 250/500/1000 x 5 seed)

Mediane sui 5 seed (per run: mediana sugli 8 osservatori base). Distanza "determinata" se |errore| < 50% di R.

| R | S | curvatura | forma | rapporto | distanza reale | % di R | determinata | direzione reale (max oss.) | distanza ideale |
|---|---|---|---|---|---|---|---|---|---|
| 250 | 1 | 0,19 | 0,20 | 0,95 | 128 m | 51% | 50% | 1,50 (9,06) | 21 m |
| 250 | 2 | 0,75 | 0,21 | 3,6 | 35 m | 14% | 100% | 1,49 (4,97) | 3,1 m |
| 250 | 3 | 1,63 | 0,21 | 7,8 | 28 m | 11% | 87,5% | 3,56 (10,72) | 1,4 m |
| 250 | 4 | 2,93 | 0,22 | 13,3 | 14 m | 5,5% | 100% | 1,66 (3,14) | 1,1 m |
| 500 | 1 | 0,10 | 0,20 | 0,50 | 774 m | indet. | 25% | 1,41 (8,87) | 65 m |
| 500 | 2 | 0,39 | 0,21 | 1,9 | 119 m | 24% | 87,5% | 1,57 (5,67) | 15 m |
| 500 | 3 | 0,87 | 0,21 | 4,1 | 102 m | 20% | 87,5% | 3,60 (6,24) | 5,5 m |
| 500 | 4 | 1,58 | 0,22 | 7,2 | 54 m | 11% | 87,5% | 1,36 (2,76) | 3,8 m |
| 1000 | 1 | 0,05 | 0,20 | 0,25 | 19 km | indet. | 12,5% | 1,19 (8,78) | 219 m |
| 1000 | 2 | 0,20 | 0,21 | 0,95 | 425 m | 42% | 50% | 1,61 (6,01) | 59 m |
| 1000 | 3 | 0,44 | 0,21 | 2,1 | 370 m | 37% | 75% | 3,51 (5,95) | 20 m |
| 1000 | 4 | 0,82 | 0,22 | 3,7 | 228 m | 23% | 75% | 1,30 (2,55) | 17 m |

- **Risultato chiave: le tre distanze cadono sulla stessa curva in funzione del rapporto curvatura / errore di
  forma.** ~1 -> 42-51%; ~2 -> 24-37%; ~4 -> 14-23%; 7-8 -> 11%; ~13 -> 5,5%. Regola di progetto per il paper.
  Figura proposta: errore di distanza relativo contro il rapporto, tutti i punti insieme.
- L'errore di forma (~0,2 m) non dipende dalla scala: la geometria stimata dallo sciame scala bene.
- **Anomalia da spiegare: a S = 3 la direzione peggiora (3,5-3,6 gradi contro 1,2-1,7) per tutte e tre le R**, quindi
  la causa e' nello sciame a quella scala, non nello spoofer. Ipotesi da verificare: interazione tra rotazione /
  oscillazione dei droni a S = 3 e ancoraggio del riferimento (8.2c), o uscita degli ospiti. Non pubblicare la figura
  della direzione prima di averla spiegata.
- Peggior osservatore: 9 gradi a S = 1, 2,6-3,1 gradi a S = 4 (sciami piu' larghi riducono gli osservatori anomali).
- **Curva unica** (figura `strada_A_curva_unica.png`, mediane per (R, S)): errore di distanza relativo contro il rapporto
  curvatura / forma. Soglia pratica ~1 (sotto: indeterminata). Pendenza log-log ~ -0,75 per rapporto >= 0,9. I punti a
  R = 1000 m stanno un po' sopra a parita' di rapporto (23% contro 14% a ~3,5-3,8): la distanza dipende SOPRATTUTTO dal
  rapporto, non solo.
- **Rilettura dell'anomalia a S = 3 (dati per seed):** direzione mediana sulle 3 R per seed 1-5 -> S=1: 1,29 0,84 7,51
  1,90 1,41; S=2: 0,44 1,57 8,19 2,72 1,00; S=3: 1,69 3,60 7,02 3,62 1,96; S=4: 1,49 1,29 5,29 0,79 1,36. Il **seed 3 e'
  pessimo a tutte le scale** (5-8 gradi; la sua costellazione ha geometria verticale peggiore, vAcc ~1,6 m contro ~0,6).
  Le 3 R non sono campioni indipendenti (stesso sciame a parita' di seed e S): 5 campioni veri per S, mediana rumorosa.
  S = 3 e' peggio di S = 2 in 4 seed su 5 ma di 1-2 gradi: probabile rumore, da confermare con piu' seed.
- **Origine dell'errore di direzione (seed 3, S = 1, 8 osservatori):** la rotazione attorno alla verticale
  dell'immagine dello sciame stimata (Kabsch, mediana durante la cattura) e' correlata con l'errore di azimut (-0,69,
  segno atteso), ma tolta la rotazione resta un residuo mediano di 4,6 gradi. La rotazione non e' solo ereditata dal GPS
  prima dell'attacco: cambia durante la cattura (es. oss. 5: -21,7 gradi all'inizio, +3,4 in mediana). **La direzione e'
  limitata soprattutto da quanto bene lo sciame conosce il proprio orientamento (bussola/IMU, 8.2c) + un residuo da
  spiegare.** Possibili miglioramenti: ancoraggio dell'orientamento piu' forte; stima della direzione in ogni secondo
  nel riferimento di quel secondo e poi media.

### 4.19 Direzione dello spoofer: fusione tra droni (S = 1, R = 500 m, 5 seed, tutto reale)

`python3 tools/direction_methods.py runs/<etichetta>/seed<N>` confronta tre metodi sulle stesse misure.

| metodo | errore (mediana sui 5 seed) | per seed | caso peggiore |
|---|---|---|---|
| singolo drone, cumulativo (attuale) | 1,47 gradi (mediana sui droni) | 1,47 / 0,82 / 7,50 / 1,83 / 1,29 | 14,7 gradi (un drone, seed 3) |
| singolo drone, per secondo (onda piana) | 2,99 gradi | 1,19 / 3,47 / 4,53 / 2,99 / 1,00 | 11,5 gradi -> SCARTATO |
| **fusione: mediana sugli 8 droni base (cumulativo)** | **0,34 gradi** | -0,34 / -0,01 / -5,81 / -1,83 / +0,20 | 5,8 gradi (seed 3) |
| fusione del per secondo | 2,98 gradi | -0,04 / +3,47 / -3,65 / -2,98 / -0,47 | 3,65 gradi |

- **La fusione e' la stima dello sciame da usare nel paper**: in 3 seed su 5 sotto 0,35 gradi; elimina i droni anomali.
  Realistica: ogni drone annuncia la propria direzione (pochi byte), lo sciame prende la mediana.
- Il metodo per secondo ignora la curvatura e introduce un errore sistematico che dipende dalla disposizione dei droni:
  risultato negativo, da citare solo come tale.
- **Limite residuo: errore di orientamento COMUNE a tutto lo sciame** (seed 3: -5,8 gradi; seed 4: -1,8): tutti i droni
  sbagliano nello stesso verso e la mediana non lo toglie. Dipende da bussola e GPS prima dell'attacco.
- `aperture_sweep.py` ha la colonna `reale_azimut_fusione_gradi`.
- **CORREZIONE (campagna `aperture2` di Jacopo, 60 run, 5 seed): la fusione aiuta SOLO nello sciame compatto.**

| R | S = 1 fusione / singolo | S = 2 | S = 3 | S = 4 |
|---|---|---|---|---|
| 250 m | 0,53 / 1,50 gradi | 1,49 / 1,49 | 2,94 / 3,56 | 1,66 / 1,66 |
| 500 m | 0,30 / 1,41 | 1,57 / 1,57 | 3,60 / 3,60 | 1,36 / 1,36 |
| 1000 m | 1,19 / 1,19 | 1,61 / 1,61 | 3,51 / 3,51 | 1,30 / 1,30 |

  A S >= 2 fusione = singolo drone: |mediana degli errori con segno| = mediana dei moduli quando tutti gli 8 droni
  sbagliano dallo stesso lato -> **errore di orientamento COMUNE a tutto lo sciame** (1,3-3,6 gradi), che la mediana non
  toglie. "0,34 gradi" vale solo per S = 1 (R = 250-500 m). Non ancora spiegato perche' sia piu' grande a S = 3.
- Tutte le altre colonne di `aperture2` sono identiche ad `aperture`: riproducibilita' confermata sul PC di Jacopo.
- **Compromesso per il paper:** sciame compatto -> direzione 0,3-0,5 gradi, distanza non misurabile a 500-1000 m;
  sciame largo (S = 3-4) -> distanza 11-23%, direzione 1,3-3,6 gradi limitata dall'orientamento comune.

### 4.20 Fase 3: radio UWB e membership robusta alle perdite (seed 1)

- `--radio=uwb` (default `wifi`, identico bit per bit). Bilancio di collegamento: EIRP -41,3 dBm/MHz su 499,2 MHz =
  -14,3 dBm (limite FCC/ETSI); spazio libero a 1 m a 6,4896 GHz = 48,7 dB; esponente 1,76 e ombreggiamento 0,83 dB
  (IEEE 802.15.4a CM5, outdoor LOS); 3 dB d'antenna; sensibilita' -100 dBm a 6,8 Mbit/s (scheda distributore: verificare
  la sez. 3.4 del datasheet DW3000). Ricezione al 50% a 85 m; con `--uwbSensDbm=-90` a 23 m (in pratica utenti DWM3000
  riportano 20-30 m, forum Qorvo). Il trasporto resta lo stack Wi-Fi di ns-3 (slot 5 ms): si decide solo SE arriva.
- **Scoperta: la membership era tarata su un Wi-Fi senza perdite.** S = 4 con radio UWB: 363 espulsioni a catena, sciame
  crollato, cattura non rilevata. Correzioni: voti con scadenza; notizie indirette (un altro drone ha misurato X); un voto
  contro X conta solo se anche chi lo riceve non ha notizie fresche di X; nessun voto contro se stessi; quorum delle
  espulsioni sui membri vivi. Con il Wi-Fi 0 espulsioni -> CSV identico.
- Risultati, radio UWB:
  - S = 1, sciame normale: 98% dei pacchetti ricevuti (<30 m 99,99%; 60-90 m 71%); metriche entro l'1% del Wi-Fi.
  - S = 4, spoofer a 500 m: 62,9% dei pacchetti; cattura e consenso 20/20; 8/8 droni base connessi fino alla fine;
    espulsioni solo tra 164 e 170 s (ospiti fuori portata), nessun base espulso da un base.
  - S = 4, drone bugiardo (nodo 0): rilevato 7/7 in 4,3 s; FA onesti dopo 0%; FA collettivi prima 0,54%; errore relativo
    base p95 5,9 m. Da confrontare con il Wi-Fi a S = 4 (Fase 10).
- **S = 2 con radio UWB (configurazione consigliata per il paper), spoofer a 500 m, seed 1, PC di Jacopo**, confrontata
  con lo stesso caso in Wi-Fi (campagna `aperture`):

| | Wi-Fi | UWB |
|---|---|---|
| pacchetti ricevuti | 100% | 91,35% (0-30 m 99,98%; 30-60 m 98,21%; 60-90 m 72,34%) |
| consenso sulla cattura | 20/20 | 20/20 |
| errore di forma | 0,198 m | 0,201 m |
| distanza reale (mediana) / determinata | 109 m (22%) / 87,5% | 45 m (9%) / 75% |
| direzione reale (mediana / peggiore) | 0,44 / 5,67 gradi | 1,43 / 10,2 gradi |

  Un solo seed: la direzione in Wi-Fi a S = 2 va da 0,44 a 8,19 gradi a seconda del seed, quindi la differenza puo' essere
  rumore. Con UWB alcune differenze di PPS non arrivano: l'insieme di misure cambia (anche l'ideale: 35 contro 41 m).
- Da fare: campagna `EXTRA="--radio=uwb"` con SCALES="1 2", DISTS="250 500 1000", SEEDS="1 2 3 4 5" (30 run) per
  confermare che la radio UWB non peggiori la direzione in modo sistematico; Fase 5 (LOS/NLOS e ombreggiamento
  correlati, errore che cresce con la distanza, riconoscimento NLOS imperfetto).
- Scelta di progetto proposta (da confermare con la campagna): **tutto UWB, sciame entro ~60 m tra i droni (S <= 2)**.
  Alternativa scartata per ora: radio ibrida (UWB per misurare, altra radio per i dati), come estensione futura.

### 4.21 Direzione peggiore con radio UWB a S = 2: causa trovata (seed 2, 3, 5, spoofer a 500 m)

Strumento: `python3 tools/observer_diag.py runs/<etichetta>/seed<N> [--per-second OSS]` (forma con sola rotazione e con
riflessione, rotazione attorno alla verticale, pacchetti ricevuti, differenze di PPS anomale, per osservatore).

- Osservatori anomali: seed 2 oss. 5 (-146,9 gradi), seed 3 oss. 7 (-145,7), seed 5 oss. 6 (+76,2; anche oss. 3 -13,0).
  **La loro geometria e' buona**: forma 0,18-0,28 m come gli altri, riflessione mai migliore della rotazione, rotazione
  4-14 gradi come gli altri, 98-99% dei pacchetti dagli altri droni base. **Ipotesi "drone periferico con geometria
  ruotata o specchiata": smentita.** (La forma mediana "invariata" era corretta: il problema non e' nella forma.)
- **Causa: sincronizzazione (9.1).** La retta tra l'orologio del mittente e quello dell'osservatore usa le ultime 20 coppie
  tx/rx SENZA limite di eta'. Con le perdite UWB verso droni lontani (soprattutto ospiti in uscita, 160-173 s) le 20 coppie
  coprono fino a ~17 s, oltre mezzo giro del contatore UWB a 40 bit (2^40 x 15,65 ps = 17,2 s; `TickDiff` legge le
  differenze in +-8,6 s): le coppie piu' vecchie vengono lette con un giro sbagliato e la differenza di PPS sbaglia di
  4-47 km. Build diagnostica, seed 2: 57 misure su 29.295 oltre 20 m di errore, 56 con finestra di 14,8-17,1 s; tutte
  le buone hanno finestra <= 8,6 s (mediana 1,9 s). Seed 3 e 5: 71 e 69 misure anomale, tutte con finestra >= 7,4 s
  (una coppia a -9 s appare a +8,2 s). Poche misure da decine di km bastano a rovinare i minimi quadrati di un osservatore.
- Spiega tutti i sintomi: Wi-Fi senza perdite -> finestra sempre ~1,9 s -> nessun problema; S = 1 -> distanze corte,
  poche perdite; errore uguale per le tre distanze dello spoofer (dipende dallo sciame); forma invariata.
  **Il bug tocca solo il log dei PPS (localizzazione dello spoofer), non il comportamento dello sciame.**
- Oracolo (coppie piu' vecchie di 4 s scartate, build non consegnata): seed 2 oss. 5 -146,9 -> -6,0 gradi; seed 3 oss. 7
  -145,7 -> -14,6; seed 5 oss. 6 +76,2 -> 0,00, oss. 3 -13,0 -> -1,1, oss. 1 +7,1 -> -1,4; tutti gli altri identici.
  Misure PPS oltre 20 m: 57-71 -> 2-8 per seed (max 27-36 m, causa ancora da capire; nessuna negli osservatori base).
- Osservazione coerente con la 4.19: dopo la correzione l'errore di direzione segue la rotazione del riferimento stimato
  (az ~ -rot_z; seed 3: rotazione 8-10 gradi, errore -8...-9,6 gradi; seed 5: rotazione entro 2 gradi, errore entro 1,5).
- **Conseguenza:** i numeri UWB a S = 2 della 4.20 (direzione 1,43 / 10,2 gradi) e della campagna UWB (3,21 gradi contro
  1,57 in Wi-Fi) sono falsati da questo bug. Rifare il confronto dopo la correzione.
- Da fare (prossimo passo): finestra delle coppie limitata nel tempo + controllo di qualita' della retta (residuo) ed
  esclusione dalla fusione delle stime incoerenti; Wi-Fi identico; campagna 5 seed Wi-Fi/UWB sul PC di Jacopo.
- Nota di metodo: guardare sempre le statistiche PER OSSERVATORE (min/max), non solo le mediane.

### 4.22 Correzione della sincronizzazione (9.1b, patch `limit the clock fit window`)

- Retta tra gli orologi: coppie al massimo di 4 s (finestra nominale 1,9 s; mezzo giro del contatore 8,6 s) e solo con una
  distanza DS-TWR di al massimo 0,3 s. Controllo di qualita': differenza di PPS scartata se il residuo RMS della retta supera
  10 m (stampa `>>> PPS:` a fine simulazione). Il controllo non e' mai scattato (0 su ~28.800 per run): le due regole
  sulle coppie bastano; resta come protezione.
- **Seconda causa trovata:** con la sola finestra di 4 s restavano 2-8 misure per seed con errore di 20-36 m; aggiungendo il
  limite di eta' della distanza spariscono. Causa: il tempo di volo sottratto a ogni coppia veniva dall'ultima distanza
  DS-TWR riuscita, vecchia anche di secondi con le perdite; l'errore e' pari a quanto si e' mosso il mittente (ospiti a
  7,5 m/s -> 30 m in 4 s).
- Identita': Wi-Fi (seed 2, S = 2, spoofer) identico in tutti i file, pps_log compreso; default (seed 1) identico; UWB S = 2
  (seed 1): CSV dello sciame, gnss_log, ground_truth identici, cambia solo pps_log. **Il bug non ha mai toccato lo sciame:
  tutte le metriche di rilevamento, forma e cattura gia' riportate restano valide.**
- UWB, S = 2, R = 500 m: misure PPS oltre 20 m 57-71 -> 0 per seed; errore di sincronizzazione std 2,56 m (atteso dal dente
  di sega 2,45 m, 4.15).

| seed | prima (mediana / peggiore) | dopo | Wi-Fi S = 2 (4.18, mediana sulle 3 R) |
|---|---|---|---|
| 1 | 1,43 / 10,2 gradi | 0,67 / 5,6 | 0,44 |
| 2 | 4,72 / 146,9 | 4,74 / 6,8 | 1,57 |
| 3 | - / 145,7 | 8,45 / 14,8 | 8,19 |
| 5 | 1,42 / 76,2 | 1,12 / 1,5 | 1,00 |

  Seed 1, distanza: 45,1 m (75% determinata) -> 44,5 m (100%). I casi da 76-147 gradi spariscono. Il seed 2 resta
  peggiore del Wi-Fi ma per l'errore di orientamento COMUNE (tutti gli osservatori dallo stesso lato, -1,8...-6,8 gradi,
  segue la rotazione del riferimento stimato, 4.19), non per un drone anomalo. Confronto Wi-Fi/UWB definitivo: campagna
  5 seed sul PC di Jacopo (`uwb_fix`).
- I numeri UWB a S = 2 della 4.20 (direzione 1,43 / 10,2 gradi; distanza 45 m / 75%) sono superati da questa sezione.
- Per il paper (sezione sul sistema): la sincronizzazione tra orologi UWB deve gestire collegamenti con perdite: finestra
  limitata nel tempo rispetto al periodo del contatore e tempo di volo fresco. Dettaglio da implementazione reale (DW3000:
  contatore a 40 bit, stesso periodo di 17,2 s).

### 4.23 Confronto Wi-Fi/UWB dopo la correzione (campagna 5 seed sul PC di Jacopo, `uwb_fix` / `wifi_fix`)

Mediane su 5 seed (seed 1-5), stessa build, stessi seed nelle due radio:

| R | S | dir. mediana UWB / Wi-Fi | dir. peggiore UWB / Wi-Fi | fusione UWB / Wi-Fi | distanza mediana UWB / Wi-Fi | determinata UWB / Wi-Fi |
|---|---|---|---|---|---|---|
| 250 | 1 | 1,66 / 1,50 | 8,10 / 9,06 | 1,15 / 0,53 | 106 / 128 m | 62,5 / 50% |
| 500 | 1 | 1,74 / 1,41 | 7,97 / 8,87 | 0,76 / 0,30 | 433 / 774 m | 25 / 25% |
| 1000 | 1 | 1,16 / 1,19 | 7,90 / 8,78 | 0,56 / 1,19 | 9.878 / 18.996 m | 12,5 / 12,5% |
| 250 | 2 | 3,31 / 1,36 | 6,21 / 4,86 | 3,31 / 1,36 | 28 / 36 m | 100 / 100% |
| 500 | 2 | 3,17 / 1,50 | 6,00 / 5,64 | 3,17 / 1,50 | 137 / 116 m | 87,5 / 87,5% |
| 1000 | 2 | 3,04 / 1,55 | 5,79 / 6,00 | 3,04 / 1,55 | 430 / 367 m | 62,5 / 62,5% |

- **Correzione efficace sugli osservatori anomali:** il peggiore in UWB a S = 2 scende da 76-147 gradi a <= 6,2 gradi, come
  in Wi-Fi. Wi-Fi invariato rispetto alla campagna precedente (atteso: la correzione non lo tocca).
- **S = 1:** UWB ~ Wi-Fi in direzione e distanza.
- **S = 2: la direzione mediana in UWB resta ~2x il Wi-Fi (3,0-3,3 contro 1,4-1,6 gradi), ma la mediana coincide con la
  fusione:** tutti gli osservatori sbagliano dallo stesso lato = errore di orientamento COMUNE del riferimento stimato
  (4.19), non droni anomali. La distanza e' equivalente tra le due radio (stessa frazione determinata).
- **Oracolo (seed 2, S = 2, R = 500 m, eseguito da Claude):**

| | dir. mediana / peggiore | rotazione del riferimento (8 osservatori) |
|---|---|---|
| UWB normale (91% dei pacchetti) | 4,74 / 6,8 gradi | 3,1-8,2 gradi |
| UWB senza perdite (`--uwbSensDbm=-130`, 100%) | 1,50 / 3,35 | 1,7-4,5 |
| Wi-Fi (CORRETTO in 4.25: la run da 1,54 / 3,33 era troncata) | 1,50 / - | identico a UWB senza perdite |

  Stessa radio e stesse temporizzazioni, cambia solo chi riceve: **il divario residuo a S = 2 e' causato dalle perdite di
  pacchetti, che fanno ruotare di piu' il riferimento stimato.** UWB senza perdite = Wi-Fi.
- Da capire (prossimo passo proposto, prima della Fase 5): QUALI perdite contano (collegamenti base-ospite lunghi,
  60-90 m al 72%? velocita' IMU nei pacchetti dell'8.2c? filtro su se stessi?) e se e' un limite dello stimatore
  (correggibile) o informativo (meno vincoli = piu' deriva di orientamento). Se fisico, e' un risultato per il paper:
  allargare la formazione rende la distanza osservabile ma, con portata UWB reale, costa precisione di direzione.
- Nota di metodo: con 5 seed la mediana per configurazione e' fragile (dipende da 1-2 seed); per il paper riportare
  anche i confronti appaiati per seed e piu' seed (>= 10, sez. 8).

### 4.24 Perche' le perdite UWB peggiorano la direzione a S = 2: rotazione del riferimento (seed 2, S = 2, R = 500 m)

Misura: rotazione attorno alla verticale del riferimento stimato da ogni osservatore base, solo droni base (Kabsch per
secondo), mediana tra osservatori e secondi. Ora registrata da `aperture_sweep.py` (colonne `rotazione_*`).

| | rotazione con gli ospiti (105-160 s) | dopo l'uscita (180-240 s) |
|---|---|---|
| UWB, tutte le perdite | 5,03 gradi | 9,32 |
| oracolo: solo gli ospiti perdono (osservatori base senza perdite) | 4,10 | 6,89 |
| oracolo: solo gli osservatori base perdono | 3,71 | 6,55 |
| UWB senza perdite (`--uwbSensDbm=-130`) | 2,61 | 5,98 |

- **Non e' caso:** l'andamento nel tempo e' lo STESSO con e senza perdite (sale fino a ~140 s, scende verso 160 s, risale
  a 190 s, scende a 210 s, risale a 230 s, fino a ~10 gradi anche senza perdite). Errore sistematico legato al moto.
  Le perdite lo amplificano (circa +2,4 gradi con gli ospiti, +3,3 dopo).
- La velocita' angolare vera della formazione base e' costante (-2,88 gradi/s): non spiega l'andamento; con omega costante
  un ritardo di inseguimento e un errore di orientamento fisso non si distinguono (ipotesi "ritardo" NON dimostrata).
- Gli osservatori base ricevono il 98-99% dei pacchetti (dai base e dagli ospiti); il 91% complessivo comprende i
  collegamenti lunghi tra ospiti.
- **Attribuzione** (con gli ospiti, fase piu' pulita: dopo l'uscita l'errore e' uno stato ereditato): perdite degli
  ospiti +1,49, perdite dirette dei base +1,10, insieme +2,42: quasi additive, **nessun colpevole unico**. Anche l'1-2%
  di pacchetti persi dai base pesa: l'orientamento dello sciame sotto cattura e' debolmente vincolato (le distanze UWB
  non lo vedono; lo ancorano solo le velocita' di bordo dell'8.2c), quindi ogni informazione persa conta.
- **Interpretazione per il paper (da confermare su piu' seed):** il limite principale della direzione non sono le perdite
  ma l'osservabilita' dell'orientamento; le perdite UWB lo amplificano. Con una formazione piu' larga la distanza diventa
  osservabile (4.18) ma i collegamenti lunghi perdono pacchetti e la direzione peggiora: compromesso da mostrare.
- **Da fare:** campagna 10 seed (Wi-Fi, UWB, UWB senza perdite) a S = 2, R = 500 m, per verificare in generale (tutto
  visto su 1 seed). Poi decidere: (a) limite dichiarato; (b) ancoraggio dell'orientamento. Nella realta' una velocita'
  nord/est dai sensori di bordo richiede gia' una rotta (bussola/magnetometro), quindi usare anche la rotta di ogni drone
  sarebbe coerente con le ipotesi dell'8.2c. Va pero' modellato il suo errore (disturbi magnetici dei motori, ipotesi da
  documentare con fonti).

### 4.25 Campagna 10 seed sulla rotazione (S = 2, R = 500 m; PC di Jacopo) e causa della rotazione

Configurazioni: Wi-Fi, UWB, UWB senza perdite (`--uwbSensDbm=-130`); colonne `rotazione_mediana_gradi` e
`reale_azimut_fusione_gradi` di `aperture_sweep.py`.

| seed | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | mediana |
|---|---|---|---|---|---|---|---|---|---|---|---|
| rotazione Wi-Fi | 1,99 | 3,46 | 8,35 | 2,77 | 1,56 | 3,38 | 3,44 | 1,36 | 7,57 | 6,97 | 3,41 |
| rotazione UWB | 2,52 | 6,09 | 9,53 | 2,80 | 1,91 | 3,68 | 6,33 | 1,75 | 11,22 | 6,94 | 4,89 |
| direzione Wi-Fi | 0,09 | 1,50 | 7,98 | 2,61 | 0,05 | 3,05 | 3,98 | 0,24 | 6,60 | 8,07 | 2,83 |
| direzione UWB | 0,66 | 4,74 | 8,45 | 3,17 | 0,07 | 3,45 | 5,32 | 0,99 | 7,72 | 10,13 | 4,09 |

- **UWB senza perdite = Wi-Fi in tutti i 10 seed** (identici al centesimo; riprodotto da Claude sul seed 2 con lo script
  della campagna: 3,46 e 1,502 gradi). La radio UWB del simulatore filtra i pacchetti sullo stesso trasporto: senza
  perdite coincide col Wi-Fi. **L'unica differenza tra le radio sono le perdite.**
- **Correzione:** il valore Wi-Fi seed 2 "1,54 / 3,33 gradi" della 4.23 veniva da una run interrotta da un timeout
  (container lento, riga "completato" assente): il valore giusto e' 1,50.
- **Perdite UWB, test appaiati:** rotazione peggiore in 9/10 seed (Wilcoxon p = 0,004; differenza mediana +0,46,
  media +1,19, max +3,65 gradi); direzione peggiore in 10/10 (p = 0,002; mediana +0,66 gradi). Effetto reale ma secondario.
- **La direzione e' la rotazione del riferimento:** 20 run, Pearson r = 0,90, Spearman 0,92,
  direzione ~ 1,00 x rotazione - 0,76 gradi.
- **Il limite dominante c'e' anche in Wi-Fi:** 3 seed su 10 (3, 9, 10) sbagliano la direzione di 6,6-8,1 gradi. La
  mediana su 10 seed (2,83 gradi in Wi-Fi) e' peggiore di quella su 5 (1,50): la campagna a 5 seed era ottimista.
- **Causa della rotazione (oracoli, seed 9, Wi-Fi):** gli errori LENTI del sensore di velocita' di bordo (`ImuModel.h`:
  bias 0,3 m/s con tau 30 s, scala 3% fissa per drone, bussola 3 gradi con tau 120 s), che il filtro tratta come rumore
  bianco (approssimazione dichiarata nel codice, stessa lezione della 6.2). Integrati, spostano le posizioni; le distanze
  UWB correggono la forma ma non la rotazione comune. Stesso seme in tutte le varianti -> stesso andamento con e senza
  perdite (spiega la 4.24).

| seed 9, Wi-Fi | rotazione | direzione | forma |
|---|---|---|---|
| default | 7,57 | 6,60 | 0,21 m |
| senza bussola (`--imuHeadingDeg=0`) | 4,20 (p95 47,9: anomalo, da verificare) | 3,88 | 0,21 m |
| senza errori lenti (`--imuVelBias=0 --imuScale=0 --imuHeadingDeg=0`) | **0,59** | **0,38** | **0,08 m** |

- **Ritirata la proposta "ancorare con la bussola" (4.24):** la bussola e' gia' nel modello e orienta il singolo drone,
  non la geometria tra i droni. Per osservare la rotazione della formazione servirebbe una misura di angolo tra droni
  (es. AoA/PDoA UWB + rotta), da valutare con fonti.
- **Conseguenze per il paper:** (1) la precisione di direzione dipende dalle ipotesi sul sensore di velocita' di bordo,
  oggi "prudenti" ma NON documentate con fonti: servono fonti (optical flow, bussola su droni) e un'analisi di
  sensibilita' (direzione contro entita' degli errori lenti); (2) possibili miglioramenti: stima della direzione nei
  primi secondi dopo la cattura (rotazione ancora piccola: l'istantanea parte dal GPS buono), oppure modellare gli errori
  lenti nel filtro; (3) le perdite UWB sono un effetto secondario da riportare (test appaiati sopra).

### 4.26 Fase 5: errore UWB realistico (`--uwbError=real`, patch `add realistic uwb ranging error`)

Modello (default `legacy` identico byte per byte: default seed 1 e UWB S = 2 con spoofer seed 1, 4 file su 4):
- **Stato LOS/NLOS correlato** con la procedura di consistenza spaziale di 3GPP TR 38.901 (par. 7.6.3.1 e 7.6.3.3):
  stato LOS = confronto tra una variabile spazialmente correlata e P_LOS(d). Per collegamento non orientato (stesso stato
  nei due versi), funzione dello spostamento dei due estremi; correlazione 1/e a `--uwbCorrDist` (default 10 m = valore
  3GPP indoor, il piu' piccolo; 3GPP non copre collegamenti tra droni: IPOTESI). P_LOS(d) = exp(-d/500) storica (IPOTESI).
  Processo a nodi interpolati (non Gauss-Markov del primo ordine: con quello la durata degli episodi dipendeva dalla
  frequenza dei pacchetti, 3,7 m -> 0,4 m di spostamento passando da 1 m a 1 cm di passo; a nodi ~11 m sempre).
- **Bias NLOS fisso per episodio**, U(0,3; 2,5) m + jitter 0,5 m per messaggio: valori ereditati dal codice originale,
  SENZA fonte (da sostituire con misure reali o letteratura).
- **Rumore che cresce con la distanza:** sigma^2 = 0,10^2 + 0,25^2 x 10^(-margine/10), margine dal bilancio della Fase 3.
  Forma motivata dal limite di Cramer-Rao del ToA (varianza ~ 1/SNR; Gezici et al., IEEE Signal Processing Magazine 2005)
  e da riscontri pratici su DW1000 (errore quasi costante fino al limite di portata, poi in crescita: forum Qorvo,
  fonte debole). sigma_bordo = 0,25 m = IPOTESI.
- **Riconoscimento NLOS imperfetto:** falsi NLOS 5% per messaggio (indipendenti), episodi NLOS non rilevati 10% (per
  intero: caso peggiore per il filtro). Coerente con accuratezze riportate per DW1000 dal 73% a oltre il 99% a seconda di
  metodo e scenario (circa 95% in dinamico, Cwalina et al.; misure indoor). Il chip stima l'NLOS dalla differenza tra
  potenza ricevuta e potenza del primo cammino (Decawave APS006 parte 3: < 6 dB probabile LOS, > 10 dB probabile NLOS).
- Filtri: varianza dal bit DICHIARATO e dalla distanza misurata; NLOS = secondo momento del bias (conservativo).

Risultati:
- Verifica nel simulatore (seed 1, UWB, S = 2, spoofer): NLOS 7,4% dei messaggi, episodi medi 2,3 s, falsi NLOS 5,0%,
  NLOS non rilevati 8,5% delle distanze; rumore LOS 7,3 cm (0-15 m) -> 18,8 cm (90-105 m); bias NLOS medio 1,3 m.
- **Forma 0,201 -> 0,302 m (+50%)**, consenso sulla cattura invariato (20/20). Oracoli (seed 1, stesso codice dello
  sciame, la correzione 9.1b tocca solo pps_log): rivelatore ideale 0,244 m, nessun NLOS 0,240 m -> circa 0,06 m dagli
  episodi NLOS NON rilevati (bias persistente trattato come rumore), circa 0,04 m da rumore con la distanza e falsi NLOS.
- S = 1, Wi-Fi, attacco interno (seed 1): rilevamento invariato (7/7, stessa latenza); chi-quadro oltre il 99,9%
  0,17% -> 0,24%; errore relativo base p95 1,77 -> 1,89 m.
- Direzione e distanza (seed 1): direzione 0,67 -> 0,37 gradi, distanza 44,5 m (100%) -> 29,8 m (87,5%). Su un seed
  NON significativo: dominate dalla deriva dei sensori di bordo (4.25). Campagna 10 seed sul PC di Jacopo.

## 5. Threat model

- **(a) Spoofer GNSS esterno** (caso principale): singola antenna, trasmette segnali GNSS falsi, non
  partecipa al TDMA. Rilevamento + localizzazione via TDoA sui clock bias.
- **(b) Drone dello sciame che mente** (insider): rilevato dalla discrepanza stima/GPS, escluso dalle ancore.
- **(c) Attaccante sulle misure UWB** (iniezione di pacchetti, riduzione della distanza, jamming): fuori
  perimetro; difesa esistente: STS di IEEE 802.15.4z. Citare gli attacchi fisici di riduzione della
  distanza su UWB.
- Limite strutturale noto: uno spoofer coordinato che sposta tutto lo sciame con lo stesso offset lento
  preserva tutte le distanze → invisibile a qualunque difesa basata solo su misure relative (serve un
  riferimento assoluto). Dichiararlo.
- Voti non autenticati: un insider potrebbe accusare droni onesti per farli escludere come ancore.

## 6. Limiti attuali (cosa noterebbe un revisore)

- Radio: i pacchetti viaggiano su Wi-Fi 802.11g a 30 dBm (tutti sentono tutti); va sostituito (Fase 3).
- Canale UWB: LOS/NLOS estratto indipendentemente a ogni pacchetto, senza correlazione (Fase 5).
- Bit LOS ideale (viene dal canale): un rilevatore NLOS reale sbaglia a volte.
- Attacco unico: 15 m in 10 s sul nodo 0, fermo al centro (geometria piu' favorevole possibile).
  Servono attacchi lenti/sottili, piu' attaccanti, bersagli in movimento ai bordi.
- Test chi-quadro calibrato (6.2) ma verificato su 3 seed e un solo scenario d'attacco; il modello di errore
  GNSS (parametri, Gauss-Markov) e' un'ipotesi da giustificare con fonti e analisi di sensibilita'.
- q_acc unico per droni lenti e veloci; sensibilita' da ricontrollare con attacchi sottili.
- Range condivisi a rotazione (10 per pacchetto): ogni range esce ogni ceil(N/10) pacchetti; da studiare la
  scalabilita' (eta' dei range contro N).
- `setSpeed`/`setScenary` ignorati: nessuno sweep di velocita' possibile finche' non si sistemano.
- Nessun dato reale.

## 7. Letteratura da citare (da ricontrollare titoli, autori e anni prima della scrittura)

- Confronto distanze da GPS contro ranging IR-UWB tra membri dello sciame (arXiv 2301.12766, 2023).
- Uso del ranging UWB tra droni per il rilevamento cooperativo dello spoofing (Frontiers, 2026).
- Residuo geometrico sui lati tra distanze misurate e GPS dichiarati (arXiv 2605.26876, 2026).
- Attacco "rigido" che preserva tutte le distanze tra droni (arXiv 2608.06885, agosto 2026).
- F. Shan et al., ranging UWB per sciami dai timestamp dei messaggi dati, Crazyflie + DW1000, 9 droni
  (IEEE INFOCOM 2021; versione per reti dinamiche e dense su IEEE/ACM ToN).
- MAC UWB con sincronizzazione tipo Glossy, 30 Hz per vicino con 13 nodi (Richter et al., 2025).
- Localizzazione di uno spoofer con TDoA da piu' ricevitori: QSSL, "A Flexible GNSS Spoofer Localization System"
  (NAVIGATION 69(1), navi.511, 2022); Babcock-Chi et al., TDOA spoofing source localization (ION GNSS+ 2025,
  Stanford); Bhamidipati & Gao, "GPS Multireceiver Joint Direct Time Estimation and Spoofer Localization" (IEEE TAES
  55(4), 2019); Broumandan 2015; Gamba 2016; "GNSS Spoofing Network Monitoring Based on Differential Pseudorange"
  (Sensors 2016). Tutti con ricevitori fermi/noti o segnali autentici visibili.
- Geolocalizzazione di uno spoofer dalla deriva di clock del ricevitore ingannato (UT Austin).
- Tippenhauer et al., requisiti per uno spoofing GPS riuscito (ACM CCS 2011): una sola antenna fa vedere
  a piu' ricevitori la stessa posizione.
- **Attacco esterno, paper:** Kerns, Shepard, Bhatti, Humphreys, "Unmanned Aircraft Capture and Control via GPS Spoofing",
  J. Field Robotics, 2014; Shepard, Bhatti, Humphreys, Fansler, "Evaluation of Smart Grid and Civilian UAV Vulnerability
  to GPS Spoofing Attacks", ION GNSS 2012; Shepard, Bhatti, Humphreys, "Drone Hack", GPS World 2012 (la cattura reale e'
  graduale: fasi di codice allineate entro pochi metri a quelle vere); Humphreys et al., "Assessing the Spoofing Threat:
  Development of a Portable GPS Civilian Spoofer", ION GNSS 2008 / GPS World 2009; Bhatti, Humphreys, "Hostile Control of
  Ships via False GPS Signals: Demonstration and Detection", NAVIGATION 2017 (anno/sede da verificare su Scholar).
- **Attacco esterno, casi reali:** White Sands 2012 (UAV catturato da ~1 km, su invito del DHS); yacht "White Rose of
  Drachs" 2013; Mar Nero 2017 (>20 navi collocate tutte all'aeroporto di Gelendzhik: stessa posizione falsa per tutti,
  rapporto US Maritime Administration del 22/6/2017); aviazione 2024 (centinaia di incidenti in Medio Oriente e Sud-est
  asiatico; +400% secondo OPSGROUP).
- **Modello radio UWB:** IEEE 802.15.4a channel model final report (Molisch et al., doc. 15-04-0662), CM5 outdoor LOS.
- IEEE 802.15.4z (STS) e attacchi di riduzione della distanza su UWB HRP.
- Misure ritardate / out-of-sequence nei filtri di Kalman (letteratura tracking, es. Bar-Shalom).

## 8. Esperimenti da fare (Fase 10, dopo le Fasi 3 e 5)

- ROC (rilevamento contro falsi allarmi) al variare della soglia; latenza di rilevamento.
- Attacchi: salto, rampa lenta sotto soglia, piu' attaccanti, bersagli mobili, spoofer esterno a
  distanze e posizioni diverse; offset rigido dello sciame (per mostrare il limite).
- Ablazioni: `--ranging=toa` contro `dstwr`; con/senza compensazione del ritardo; con/senza esclusione
  delle ancore; q_acc.
- Scalabilita': numero di droni, dimensione del pacchetto, durata del frame, k vicini.
- Localizzazione spoofer: errore di direzione e di distanza contro distanza dallo spoofer, numero di
  droni, apertura della formazione, tempo di osservazione.
- Baseline dalla letteratura: confronto a coppie distanza-GPS contro distanza-UWB (arXiv 2301.12766).
- Piu' seed (almeno 10) per le figure finali.

## 9. Ordine dei lavori

1. Fasi 6 → 7 → 8 → 9 (ricevitore GNSS, spoofer, rilevamento, localizzazione): contributo principale.
2. Fasi 3 + 5 insieme (radio e canale realistici).
3. Fase 10 (esperimenti). **Le figure finali solo dopo le Fasi 3 e 5.**
4. Scrittura in parallelo dalla settimana 5.

## 10. Per riprendere in una nuova chat con Claude

Incollare questo file e `ROADMAP.md`, poi dire: "riprendi dalla prossima fase". Contesto da dare:

- Repository: https://github.com/jacopodilauro/decentralized-multilateralization-drone-swarm, branch
  `feature/realistic-simulator`. Jacopo compila in `~/ns-3-dev` con `./ns3 build`, eseguibile
  `/home/jacopo/ns-3-dev/build/distry_mlat26v4-nomanager`, sorgenti in `~/pleion/src`, Linux Mint.
- Ambiente di Claude (ricrearlo): `pip install ns3 --break-system-packages` (ns-3.44, verificato
  identico all'ns-3-dev di Jacopo su tutte le metriche), `apt-get install -y libeigen3-dev`, clonare il
  branch, `NS3_PREFIX=$(python3 -c "import importlib.util;print(importlib.util.find_spec('ns3').submodule_search_locations[0])") tools/build_standalone.sh sim`.
  I CSV pesano ~100 MB a simulazione: cancellare quelli degli esperimenti dopo l'uso.
- Regole di lavoro concordate:
  - rispondere in italiano; spiegare cosa, perche', come e cosa aspettarsi;
  - **nessun codice senza averlo compilato, eseguito e verificato** (regressione con `run_seeds.sh` +
    `compare.py`, test isolati con `run_tests.sh`, esperimenti oracolo per trovare le cause);
  - commit con autore **Jacopo Di Lauro <jacopo.dilauro04@gmail.com>**, messaggi in inglese corti e semplici;
  - consegnare patch (`git format-patch`), verificate applicandole su un clone pulito del branch GitHub;
  - si possono unire piu' passi in una patch quando sono legati;
  - attenzione agli spazi a fine riga ereditati e ai file con CRLF (usare patch binaria se serve);
  - con `--netanim=1` il CSV differisce per arrotondamento: per gli esperimenti sempre `--netanim=0`.
- Stato (20/10): su GitHub tutto fino alla Fase 3 (radio UWB opzionale `--radio=uwb`, default Wi-Fi identico; membership
  robusta alle perdite). Trovata la causa della direzione peggiore in UWB a S = 2 (4.21: finestra della retta tra orologi oltre mezzo
  giro del contatore a 40 bit). Correzione consegnata (4.22) e confronto fatto (4.23): a S = 2 resta un errore di orientamento comune
  causato dalle perdite UWB (oracolo). Diagnosi fatta (4.24: orientamento debolmente vincolato,
  amplificato dalle perdite). Campagna 10 seed fatta (4.25): la rotazione viene dagli errori lenti dei
  sensori di bordo; perdite UWB effetto secondario. Fase 5 consegnata (4.26). Simulatore: restano sensori di bordo con fonti,
  configurazione del paper, punti minori, verifica finale (lista in cima alla ROADMAP)
  (bozza gia' pronta e testata nell'ambiente di Claude, da riconsegnare dopo la correzione). Scadenze MobiSys
  2027 ufficiali ancora TBA; aggregatore: abstract 27/11/2026, paper 4/12/2026 AoE (l'anno scorso 28/11 e 5/12). Il
  bando chiede un abstract registrato di almeno 100 parole, coerente con quello finale. Parlare col relatore della
  prova hardware (UWB + PPS, GNSS solo via cavo). Aperti: errore di orientamento comune nella direzione (4.19).
- Quando si riprende in una nuova chat: aggiornare nel progetto Claude PAPER_NOTES.md e ROADMAP.md (prenderli dal repo).
- Dopo `git am` di una patch che tocca il C++: SEMPRE `cd ~/ns-3-dev && ./ns3 build` prima di simulare (il 15/10 una
  campagna e' fallita per una build vecchia). Il .gitignore ora ignora *.csv, *.png, *.xml, *.svg: per salvare figure o
  risultati nel repo usare `git add -f`.
- Lezione di metodo: guardare sempre anche p99 e massimo, non solo p95 (le divergenze brevi stavano sotto il p95).
- Ambiente di Claude: una sola CPU, i processi in background vengono fermati tra un turno e l'altro. Lanciare
  le simulazioni in primo piano e cancellare subito i CSV. Le campagne a 3+ seed conviene farle sul PC di Jacopo.
