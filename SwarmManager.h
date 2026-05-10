#ifndef SWARM_MANAGER_H
#define SWARM_MANAGER_H

#include <vector>
#include <queue>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

// ---------------------------------------------------------------------------
// SwarmEvent: descrive un singolo evento join o leave
// ---------------------------------------------------------------------------
struct SwarmEvent {
    enum Type { JOIN, LEAVE };

    Type     type;
    double   time;       // tempo simulazione [s]
    uint32_t droneId;    // ID del drone coinvolto
};

// ---------------------------------------------------------------------------
// SwarmManager: gestisce il pool di ID riutilizzabili e la lista eventi
//
// Opzione B — ID riutilizzabili:
//   - Ogni drone ha un ID fisso assegnato alla creazione del nodo ns-3.
//   - Quando un drone esce, il suo ID torna nel pool (free_ids).
//   - Il prossimo join pesca dal pool il primo ID libero.
//   - Il TDMA slot è sempre indicizzato sull'ID → nessun riordino.
//
// Formato stringa CMD:
//   --join="10:0,50:0"    → al t=10s entra un drone (ID pescato dal pool),
//                            al t=50s ne entra un altro
//   --leave="80:2,120:1"  → al t=80s esce il drone con ID 2,
//                            al t=120s esce quello con ID 1
//
// Nota: nel --join il campo dopo ':' è ignorato (ID assegnato dal pool).
//       Nel --leave il campo dopo ':' è l'ID esatto del drone che esce.
// ---------------------------------------------------------------------------
class SwarmManager {
public:
    // ----------------------------------------------------------------------
    // Init: nDronesBase = droni già presenti all'avvio (ID 0..nDronesBase-1)
    //       nGuestSlots = quanti slot extra pre-allocare in ns-3 per gli ospiti
    // ----------------------------------------------------------------------
    void Init(uint32_t nDronesBase, uint32_t nGuestSlots) {
        m_nBase = nDronesBase;
        m_totalSlots = nDronesBase + nGuestSlots;

        // Gli slot ospiti sono tutti disponibili all'inizio
        for (uint32_t i = nDronesBase; i < m_totalSlots; ++i)
            m_freeIds.push(i);

        m_activeIds.clear();
        for (uint32_t i = 0; i < nDronesBase; ++i)
            m_activeIds.push_back(i);
    }

    // ----------------------------------------------------------------------
    // Parsing "--join=10:0,50:0" → lista di eventi JOIN ordinati per tempo
    // ----------------------------------------------------------------------
    void ParseJoin(const std::string& s) {
        ParseEvents(s, SwarmEvent::JOIN);
    }

    // ----------------------------------------------------------------------
    // Parsing "--leave=80:2,120:1" → lista di eventi LEAVE ordinati per tempo
    // ----------------------------------------------------------------------
    void ParseLeave(const std::string& s) {
        ParseEvents(s, SwarmEvent::LEAVE);
    }

    // ----------------------------------------------------------------------
    // Assegna un ID dal pool per un nuovo join.
    // Ritorna UINT32_MAX se il pool è esaurito.
    // ----------------------------------------------------------------------
    uint32_t AssignId() {
        if (m_freeIds.empty()) {
            std::cerr << "[SwarmManager] ERRORE: pool ID esaurito! "
                      << "Aumenta nGuestSlots." << std::endl;
            return UINT32_MAX;
        }
        uint32_t id = m_freeIds.front();
        m_freeIds.pop();
        m_activeIds.push_back(id);
        std::sort(m_activeIds.begin(), m_activeIds.end());
        return id;
    }

    // ----------------------------------------------------------------------
    // Rilascia un ID al pool dopo un leave.
    // ----------------------------------------------------------------------
    void ReleaseId(uint32_t id) {
        if (id < m_nBase) {
            std::cerr << "[SwarmManager] ERRORE: tentativo di rilasciare "
                      << "un ID base (" << id << "). Ignorato." << std::endl;
            return;
        }
        m_freeIds.push(id);
        m_activeIds.erase(
            std::remove(m_activeIds.begin(), m_activeIds.end(), id),
            m_activeIds.end());
    }

    // ----------------------------------------------------------------------
    // Getters
    // ----------------------------------------------------------------------
    const std::vector<SwarmEvent>& GetEvents()    const { return m_events; }
    const std::vector<uint32_t>&   GetActiveIds() const { return m_activeIds; }
    uint32_t                       GetTotalSlots() const { return m_totalSlots; }

    // Quanti join sono stati programmati (= quanti slot ospiti servono)
    uint32_t CountJoins() const {
        uint32_t n = 0;
        for (auto& e : m_events)
            if (e.type == SwarmEvent::JOIN) ++n;
        return n;
    }

    void PrintEvents() const {
        std::cout << "[SwarmManager] Eventi programmati:" << std::endl;
        for (auto& e : m_events) {
            std::cout << "  t=" << e.time << "s  "
                      << (e.type == SwarmEvent::JOIN ? "JOIN" : "LEAVE")
                      << "  droneId=" << e.droneId << std::endl;
        }
    }

private:
    uint32_t              m_nBase       = 0;
    uint32_t              m_totalSlots  = 0;
    std::queue<uint32_t>  m_freeIds;
    std::vector<uint32_t> m_activeIds;
    std::vector<SwarmEvent> m_events;

    // ----------------------------------------------------------------------
    // Parser generico: "10:0,50:0" → eventi del tipo dato
    // Per JOIN: droneId nel CSV è ignorato, verrà assegnato dal pool
    // Per LEAVE: droneId nel CSV è l'ID esatto da rimuovere
    // ----------------------------------------------------------------------
    // ----------------------------------------------------------------------
    // Parser aggiornato:
    // Per JOIN:  "10, 50" → legge solo il tempo (ID assegnato dopo)
    // Per LEAVE: "80:8, 100:9" → legge tempo e ID esatto
    // ----------------------------------------------------------------------
    void ParseEvents(const std::string& s, SwarmEvent::Type type) {
        if (s.empty()) return;
        std::stringstream ss(s);
        std::string token;
        while (std::getline(ss, token, ',')) {
            // Ignora spazi o token vuoti
            if (token.find_first_not_of(' ') == std::string::npos) continue;

            SwarmEvent ev;
            ev.type = type;

            if (type == SwarmEvent::JOIN) {
                // Sintassi pulita: "10" o " 50"
                ev.time = std::stod(token);
                ev.droneId = UINT32_MAX; // Assegnato in main.cpp
            } else {
                // Sintassi esplicita: "80:8" o " 100:9"
                std::stringstream ts(token);
                std::string tStr, idStr;
                if (!std::getline(ts, tStr, ':')) continue;
                if (!std::getline(ts, idStr, ':')) continue;
                ev.time = std::stod(tStr);
                ev.droneId = (uint32_t)std::stoul(idStr);
            }
            m_events.push_back(ev);
        }
        // Ordina cronologicamente
        std::sort(m_events.begin(), m_events.end(),
                  [](const SwarmEvent& a, const SwarmEvent& b){
                      return a.time < b.time;
                  });
    }
};

#endif // SWARM_MANAGER_H
