#include "Trajectories.h"
#include "ns3/simulator.h"
#include "ns3/waypoint-mobility-model.h"
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace ns3;

const double ARENA_CENTER_X = 100.0;
const double ARENA_CENTER_Y = 100.0;
const double ARENA_CENTER_Z_MEAN = 50.0; 

const double CORE_SPHERE_RADIUS = 10.0; 
const double ORBIT_RADIUS = 25.0; 
const double ORBIT_SPEED = 0.2;    

const double PATROL_SPEED = 10.0;   
const double PATROL_LENGTH = 50.0; 

Vector GetFibonacciPoint(int i, int n, double radius) {
    double y = 1 - (i / (double)(n - 1)) * 2;
    double radius_at_y = std::sqrt(1 - y * y);
    double theta = M_PI * (3 - std::sqrt(5)) * i;
    double x = std::cos(theta) * radius_at_y;
    double z = std::sin(theta) * radius_at_y;
    return Vector(x * radius, y * radius, z * radius);
}

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
    int num_drones = (total_nodes > 1) ? total_nodes - 1 : 1; 
    int num_core = std::min(10, num_drones);        // Fino a 10 droni nel core
    int remaining = num_drones - num_core;
    int num_orbiters = remaining / 2;               // 50% del resto orbita
    int num_patrollers = remaining - num_orbiters;  // 50% del resto pattuglia

    for (double t = 0.0; t <= simTime; t += step_sec) {
        Vector pos;
        // 1. Master Statico (ID 0)
        if (id == 0) {
            pos = Vector(ARENA_CENTER_X, ARENA_CENTER_Y, ARENA_CENTER_Z_MEAN);
        }
        // 2. Sciame Core (Dinamico da 1-num_core)
        else if (id <= num_core) {
            Vector offset = GetFibonacciPoint(id - 1, num_core, CORE_SPHERE_RADIUS);
            pos = Vector(ARENA_CENTER_X + offset.x, ARENA_CENTER_Y + offset.y, ARENA_CENTER_Z_MEAN + offset.z);
        }
        // 3. Orbitanti "Atomo" (num_orbiters)
        else if (id <= num_core + num_orbiters) {
            int orbiterIdx = id - num_core - 1;
            double phase = orbiterIdx * (2.0 * M_PI / std::max(1, num_orbiters)); 
            double tiltAngle = orbiterIdx * (M_PI / std::max(1, num_orbiters));
            double angle = ORBIT_SPEED * t + phase;
            double r = ORBIT_RADIUS + (orbiterIdx % 4) * 2.0; 

            pos.x = ARENA_CENTER_X + r * std::cos(angle) * std::cos(tiltAngle);
            pos.y = ARENA_CENTER_Y + r * std::sin(angle);
            pos.z = ARENA_CENTER_Z_MEAN + r * std::cos(angle) * std::sin(tiltAngle);
        }
        // 4. Pattugliatori "Lobo" (num_patrollers)
        else {
            int patrolIdx = id - num_core - num_orbiters - 1;
            double A = PATROL_LENGTH / 2.0;
            double B = 10.0;
            double w = PATROL_SPEED / A;    
            double timeOffset = patrolIdx * 4.0; 
            double currentTime = t + timeOffset;
            double ex = A * std::cos(w * currentTime);
            double ey = B * std::sin(w * currentTime);
            double lineAngle = patrolIdx * (M_PI / std::max(1, num_patrollers)); 
            double rotated_x = ex * std::cos(lineAngle) - ey * std::sin(lineAngle);
            double rotated_y = ex * std::sin(lineAngle) + ey * std::cos(lineAngle);
            double z_offset = (patrolIdx % 5) * 6.0 - 12.0; 

            pos.x = ARENA_CENTER_X + rotated_x;
            pos.y = ARENA_CENTER_Y + rotated_y;
            pos.z = ARENA_CENTER_Z_MEAN + z_offset;
        }
        mob->AddWaypoint(Waypoint(Seconds(t), pos));
    }
}

/*void AssignGuestTrajectory(
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
    
    double start_radius = 100.0;
    Vector startPos(ARENA_CENTER_X + start_radius * std::cos(angle),
                    ARENA_CENTER_Y + start_radius * std::sin(angle),
                    ARENA_CENTER_Z_MEAN);

    Vector centerPos(ARENA_CENTER_X, ARENA_CENTER_Y, ARENA_CENTER_Z_MEAN);
    Vector orbitStartPos(centerPos.x + 15.0 * std::cos(angle), centerPos.y + 15.0 * std::sin(angle), centerPos.z);
                         
    double endStayTime = 150.0 - 50.0;
    double endAngle = endStayTime * 0.5 + angle;
    Vector orbitEndPos(centerPos.x + 15.0 * std::cos(endAngle), centerPos.y + 15.0 * std::sin(endAngle), centerPos.z);

    for (double t = 0.0; t <= simTime; t += step_sec) {
        Vector pos;
        if (t < 50.0) {
            double progress = t / 50.0;
            pos.x = startPos.x + (orbitStartPos.x - startPos.x) * progress;
            pos.y = startPos.y + (orbitStartPos.y - startPos.y) * progress;
            pos.z = startPos.z;
        }
        else if (t >= 50.0 && t <= 150.0) {
            double stayTime = t - 50.0;
            pos.x = centerPos.x + 15.0 * std::cos(stayTime * 0.5 + angle);
            pos.y = centerPos.y + 15.0 * std::sin(stayTime * 0.5 + angle);
            pos.z = centerPos.z;
        }
        else if (t > 150.0 && t <= 200.0) {
            double progress = (t - 150.0) / 50.0;
            pos.x = orbitEndPos.x + (startPos.x - orbitEndPos.x) * progress;
            pos.y = orbitEndPos.y + (startPos.y - orbitEndPos.y) * progress;
            pos.z = orbitEndPos.z;
        }
        else {
            pos = startPos;
        }
        mob->AddWaypoint(Waypoint(Seconds(t), pos));
    }
}*/

/*void AssignGuestTrajectory(
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
    
    // Posizione "Fuori" dal Geofence (raggio 100) e "Dentro" (raggio 15)
    Vector outPos(ARENA_CENTER_X + 100.0 * std::cos(angle),
                  ARENA_CENTER_Y + 100.0 * std::sin(angle),
                  ARENA_CENTER_Z_MEAN);
                  
    Vector inPos(ARENA_CENTER_X + 15.0 * std::cos(angle),
                 ARENA_CENTER_Y + 15.0 * std::sin(angle),
                 ARENA_CENTER_Z_MEAN);

    for (double t = 0.0; t <= simTime; t += step_sec) {
        Vector pos;
        
        // --- COREOGRAFIA MULTI-INGRESSO ---
        if (t < 30.0) { pos = outPos; } // t: 0-30 fuori
        else if (t <= 40.0) {           // t: 30-40 entrano
            double p = (t - 30.0) / 10.0;
            pos = Vector(outPos.x + (inPos.x - outPos.x)*p, outPos.y + (inPos.y - outPos.y)*p, ARENA_CENTER_Z_MEAN);
        }
        else if (t <= 70.0) { pos = inPos; } // t: 40-70 dentro (1° In)
        else if (t <= 80.0) {                // t: 70-80 escono
            double p = (t - 70.0) / 10.0;
            pos = Vector(inPos.x + (outPos.x - inPos.x)*p, inPos.y + (outPos.y - inPos.y)*p, ARENA_CENTER_Z_MEAN);
        }
        else if (t <= 110.0) { pos = outPos; } // t: 80-110 fuori
        else if (t <= 120.0) {                 // t: 110-120 ri-entrano
            double p = (t - 110.0) / 10.0;
            pos = Vector(outPos.x + (inPos.x - outPos.x)*p, outPos.y + (inPos.y - outPos.y)*p, ARENA_CENTER_Z_MEAN);
        }
        else if (t <= 150.0) { pos = inPos; }  // t: 120-150 dentro (2° In)
        else if (t <= 160.0) {                 // t: 150-160 escono
            double p = (t - 150.0) / 10.0;
            pos = Vector(inPos.x + (outPos.x - inPos.x)*p, inPos.y + (outPos.y - inPos.y)*p, ARENA_CENTER_Z_MEAN);
        }
        else if (t <= 190.0) { pos = outPos; } // t: 160-190 fuori
        else if (t <= 200.0) {                 // t: 190-200 entrano per l'attacco
            double p = (t - 190.0) / 10.0;
            pos = Vector(outPos.x + (inPos.x - outPos.x)*p, outPos.y + (inPos.y - outPos.y)*p, ARENA_CENTER_Z_MEAN);
        }
        else { pos = inPos; }                  // t: >200 rimangono dentro per votare durante l'attacco

        mob->AddWaypoint(Waypoint(Seconds(t), pos));
    }
}*/

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

    Vector outPos(ARENA_CENTER_X + 100.0 * std::cos(angle),
                  ARENA_CENTER_Y + 100.0 * std::sin(angle),
                  ARENA_CENTER_Z_MEAN);

    Vector inPos(ARENA_CENTER_X + 15.0 * std::cos(angle),
                 ARENA_CENTER_Y + 15.0 * std::sin(angle),
                 ARENA_CENTER_Z_MEAN);

    // Sfasamento individuale: ogni guest entra 0.5s dopo il precedente.
    // guestId parte da 8 (primo guest), quindi (guestId % 12) va da 0 a 11
    // => sfasamento totale tra primo e ultimo guest: 5.5s
    double tOff = (guestId % 12) * 0.5;

    for (double t = 0.0; t <= simTime; t += step_sec) {
        Vector pos;

        // tc è il tempo "percepito" da questo guest — shiftato di tOff
        // t rimane il tempo reale del waypoint ns3
        double tc = t - tOff;

        // --- COREOGRAFIA MULTI-INGRESSO (identica a prima, su tc) ---

        // t: 0-30 fuori (attesa iniziale)
        if (tc < 30.0) {
            pos = outPos;
        }
        // t: 30-40 entrano (1° ingresso)
        else if (tc <= 40.0) {
            double p = (tc - 30.0) / 10.0;
            pos = Vector(outPos.x + (inPos.x - outPos.x) * p,
                         outPos.y + (inPos.y - outPos.y) * p,
                         ARENA_CENTER_Z_MEAN);
        }
        // t: 40-70 dentro (1° sosta)
        else if (tc <= 70.0) {
            pos = inPos;
        }
        // t: 70-80 escono (1° uscita)
        else if (tc <= 80.0) {
            double p = (tc - 70.0) / 10.0;
            pos = Vector(inPos.x + (outPos.x - inPos.x) * p,
                         inPos.y + (outPos.y - inPos.y) * p,
                         ARENA_CENTER_Z_MEAN);
        }
        // t: 80-110 fuori
        else if (tc <= 110.0) {
            pos = outPos;
        }
        // t: 110-120 ri-entrano (2° ingresso)
        else if (tc <= 120.0) {
            double p = (tc - 110.0) / 10.0;
            pos = Vector(outPos.x + (inPos.x - outPos.x) * p,
                         outPos.y + (inPos.y - outPos.y) * p,
                         ARENA_CENTER_Z_MEAN);
        }
        // t: 120-150 dentro (2° sosta)
        else if (tc <= 150.0) {
            pos = inPos;
        }
        // t: 150-160 escono (2° uscita)
        else if (tc <= 160.0) {
            double p = (tc - 150.0) / 10.0;
            pos = Vector(inPos.x + (outPos.x - inPos.x) * p,
                         inPos.y + (outPos.y - inPos.y) * p,
                         ARENA_CENTER_Z_MEAN);
        }
        // t: 160-190 fuori
        else if (tc <= 190.0) {
            pos = outPos;
        }
        // t: 190-200 entrano per l'attacco (3° ingresso)
        else if (tc <= 200.0) {
            double p = (tc - 190.0) / 10.0;
            pos = Vector(outPos.x + (inPos.x - outPos.x) * p,
                         outPos.y + (inPos.y - outPos.y) * p,
                         ARENA_CENTER_Z_MEAN);
        }
        // t: >200 rimangono dentro per votare durante l'attacco
        else {
            pos = inPos;
        }

        mob->AddWaypoint(Waypoint(Seconds(t), pos));
    }
}