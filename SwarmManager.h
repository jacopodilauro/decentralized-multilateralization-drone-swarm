#ifndef SWARM_MANAGER_H
#define SWARM_MANAGER_H

#include <vector>
#include <queue>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>


struct SwarmEvent {
    enum Type { JOIN, LEAVE };

    Type     type;
    double   time;       // tempo simulazione [s]
    uint32_t droneId;    // ID del drone coinvolto
};

class SwarmManager {
public:
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

    void ParseJoin(const std::string& s) {
        ParseEvents(s, SwarmEvent::JOIN);
    }

    void ParseLeave(const std::string& s) {
        ParseEvents(s, SwarmEvent::LEAVE);
    }

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

    const std::vector<SwarmEvent>& GetEvents()    const { return m_events; }
    const std::vector<uint32_t>&   GetActiveIds() const { return m_activeIds; }
    uint32_t                       GetTotalSlots() const { return m_totalSlots; }

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
                ev.time = std::stod(token);
                ev.droneId = UINT32_MAX;
            } else {
                std::stringstream ts(token);
                std::string tStr, idStr;
                if (!std::getline(ts, tStr, ':')) continue;
                if (!std::getline(ts, idStr, ':')) continue;
                ev.time = std::stod(tStr);
                ev.droneId = (uint32_t)std::stoul(idStr);
            }
            m_events.push_back(ev);
        }
        std::sort(m_events.begin(), m_events.end(),
                  [](const SwarmEvent& a, const SwarmEvent& b){
                      return a.time < b.time;
                  });
    }
};

#endif 
