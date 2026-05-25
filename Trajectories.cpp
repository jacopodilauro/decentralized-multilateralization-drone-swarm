#include "Trajectories.h"
#include "ns3/simulator.h"
#include "ns3/waypoint-mobility-model.h"
#include <cmath>
#include <iostream>
#include <algorithm> // Necessario per std::min e std::max

using namespace ns3;

// --- PARAMETRI DELLO SPAZIO (Ora indipendenti dal numero di droni) ---

const double ARENA_CENTER_X = 100.0;
const double ARENA_CENTER_Y = 100.0;
const double ARENA_CENTER_Z_MEAN = 50.0; 

const double CORE_SPHERE_RADIUS = 10.0; 
const double ORBIT_RADIUS = 25.0; 
const double ORBIT_SPEED = 0.2;    

const double PATROL_SPEED = 10.0;   
const double PATROL_LENGTH = 50.0; 

// Funzione Helper interna
Vector GetFibonacciPoint(int i, int n, double radius) {
    double y = 1 - (i / (double)(n - 1)) * 2;
    double radius_at_y = std::sqrt(1 - y * y);
    double theta = M_PI * (3 - std::sqrt(5)) * i;
    double x = std::cos(theta) * radius_at_y;
    double z = std::sin(theta) * radius_at_y;
    return Vector(x * radius, y * radius, z * radius);
}

// ===========================================================================
// Implementazione di AssignTrajectoryToNode (Completamente Scalabile)
// ===========================================================================
void AssignTrajectoryToNode(
    ns3::Ptr<ns3::Node> node,
    int                 id,
    int                 total_nodes,
    double              simTime,
    double              speed,
    double              step_sec,
    int                 scenary
) {
    Ptr<WaypointMobilityModel> mob = node->GetObject<WaypointMobilityModel>();
    if (!mob) {
        std::cerr << "Errore: WaypointMobilityModel non trovato." << std::endl;
        return;
    }

    // --- RIPARTIZIONE DINAMICA DELLE SQUADRE ---
    // Escludiamo il Master (ID 0) dal conteggio
    int num_drones = (total_nodes > 1) ? total_nodes - 1 : 1; 

    // Ripartizione automatica
    int num_core = std::min(10, num_drones);        // Fino a 10 droni nel core
    int remaining = num_drones - num_core;
    
    int num_orbiters = remaining / 2;               // 50% del resto orbita
    int num_patrollers = remaining - num_orbiters;  // 50% del resto pattuglia
    // -------------------------------------------

    for (double t = 0.0; t <= simTime; t += step_sec) {
        Vector pos;

        // 1. Master Statico (ID 0)
        if (id == 0) {
            pos = Vector(ARENA_CENTER_X, ARENA_CENTER_Y, ARENA_CENTER_Z_MEAN);
        }
        // 2. Sciame Core (Dinamico da 1 a num_core)
        else if (id <= num_core) {
            Vector offset = GetFibonacciPoint(id - 1, num_core, CORE_SPHERE_RADIUS);
            pos = Vector(ARENA_CENTER_X + offset.x, ARENA_CENTER_Y + offset.y, ARENA_CENTER_Z_MEAN + offset.z);
        }
        // 3. Orbitanti "Atomo" (Dinamico in base a num_orbiters)
        else if (id <= num_core + num_orbiters) {
            int orbiterIdx = id - num_core - 1;
            
            // Distribuzione uniforme su 360° per le fasi e le inclinazioni
            double phase = orbiterIdx * (2.0 * M_PI / std::max(1, num_orbiters)); 
            double tiltAngle = orbiterIdx * (M_PI / std::max(1, num_orbiters));
            
            double angle = ORBIT_SPEED * t + phase;
            double r = ORBIT_RADIUS + (orbiterIdx % 4) * 2.0; 

            pos.x = ARENA_CENTER_X + r * std::cos(angle) * std::cos(tiltAngle);
            pos.y = ARENA_CENTER_Y + r * std::sin(angle);
            pos.z = ARENA_CENTER_Z_MEAN + r * std::cos(angle) * std::sin(tiltAngle);
        }
        // 4. Pattugliatori (Rotte a Ovale / Circuito fluido)
        else {
            int patrolIdx = id - num_core - num_orbiters - 1;
            
            // Dimensioni del circuito a ovale
            double A = PATROL_LENGTH / 2.0; // Raggio maggiore (Lunghezza pattugliamento)
            double B = 10.0;                // Raggio minore (Distanza tra corsia di andata e ritorno)
            
            // Frequenza angolare (calcola quanto tempo ci mette a fare un giro in base alla PATROL_SPEED)
            double w = PATROL_SPEED / A;    
            
            // Sfasamento temporale per non farli partire tutti vicini
            double timeOffset = patrolIdx * 4.0; 
            double currentTime = t + timeOffset;

            // Calcoliamo la posizione su un ovale perfetto (non ancora ruotato)
            double ex = A * std::cos(w * currentTime);
            double ey = B * std::sin(w * currentTime);

            // Angolo per la rotazione a stella (distribuisce i circuiti a 360 gradi)
            double lineAngle = patrolIdx * (M_PI / std::max(1, num_patrollers)); 

            // Matematica di rotazione 2D: ruotiamo l'intero ovale nello spazio
            double rotated_x = ex * std::cos(lineAngle) - ey * std::sin(lineAngle);
            double rotated_y = ex * std::sin(lineAngle) + ey * std::cos(lineAngle);

            // Altitudini diverse per non farli scontrare quando passano dal centro
            double z_offset = (patrolIdx % 5) * 6.0 - 12.0; 

            pos.x = ARENA_CENTER_X + rotated_x;
            pos.y = ARENA_CENTER_Y + rotated_y;
            pos.z = ARENA_CENTER_Z_MEAN + z_offset;
        }

        mob->AddWaypoint(Waypoint(Seconds(t), pos));
    }
}

// ===========================================================================
// Implementazione di AssignGuestTrajectory (Richiama la funzione base)
// ===========================================================================
// ===========================================================================
// Implementazione di AssignGuestTrajectory (In-and-Out per Test Join/Leave)
// ===========================================================================
// ===========================================================================
// Implementazione di AssignGuestTrajectory (Corretta: Volo Fluido)
// ===========================================================================
void AssignGuestTrajectory(
    ns3::Ptr<ns3::Node> node,
    int                 guestId,
    double              simTime,
    double              speed,
    int                 scenary,
    double              t_join,
    double              t_leave,
    double              step_sec
) {
    Ptr<WaypointMobilityModel> mob = node->GetObject<WaypointMobilityModel>();
    if (!mob) return;

    double angle = guestId * (2.0 * M_PI / 20.0); 
    
    // Punto di partenza: 100 metri di distanza dal centro
    double start_radius = 100.0;
    Vector startPos(ARENA_CENTER_X + start_radius * std::cos(angle),
                    ARENA_CENTER_Y + start_radius * std::sin(angle),
                    ARENA_CENTER_Z_MEAN);

    Vector centerPos(ARENA_CENTER_X, ARENA_CENTER_Y, ARENA_CENTER_Z_MEAN);

    // -------------------------------------------------------------
    // FIX: Calcoliamo i punti esatti di aggancio e sgancio dall'orbita
    // -------------------------------------------------------------
    Vector orbitStartPos(centerPos.x + 15.0 * std::cos(angle),
                         centerPos.y + 15.0 * std::sin(angle),
                         centerPos.z);
                         
    // A t=150, il drone ha trascorso 100 secondi nell'orbita. L'angolo finale è:
    double endStayTime = 150.0 - 50.0;
    double endAngle = endStayTime * 0.5 + angle;
    Vector orbitEndPos(centerPos.x + 15.0 * std::cos(endAngle),
                       centerPos.y + 15.0 * std::sin(endAngle),
                       centerPos.z);

    for (double t = 0.0; t <= simTime; t += step_sec) {
        Vector pos;
        
        // FASE 1 (0 - 50s): Avvicinamento fino al BORDO dell'orbita
        if (t < 50.0) {
            double progress = t / 50.0;
            pos.x = startPos.x + (orbitStartPos.x - startPos.x) * progress;
            pos.y = startPos.y + (orbitStartPos.y - startPos.y) * progress;
            pos.z = startPos.z;
        }
        // FASE 2 (50 - 150s): Stazionamento (Gira in tondo morbidamente)
        else if (t >= 50.0 && t <= 150.0) {
            double stayTime = t - 50.0;
            pos.x = centerPos.x + 15.0 * std::cos(stayTime * 0.5 + angle);
            pos.y = centerPos.y + 15.0 * std::sin(stayTime * 0.5 + angle);
            pos.z = centerPos.z;
        }
        // FASE 3 (150 - 200s): Allontanamento dal punto di SGANCIO dell'orbita
        else if (t > 150.0 && t <= 200.0) {
            double progress = (t - 150.0) / 50.0;
            pos.x = orbitEndPos.x + (startPos.x - orbitEndPos.x) * progress;
            pos.y = orbitEndPos.y + (startPos.y - orbitEndPos.y) * progress;
            pos.z = orbitEndPos.z;
        }
        // FASE 4 (200s in poi): Lontani e Silenziosi
        else {
            pos = startPos;
        }

        mob->AddWaypoint(Waypoint(Seconds(t), pos));
    }
}