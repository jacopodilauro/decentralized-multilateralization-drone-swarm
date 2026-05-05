#include "Trajectories.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <functional>

using namespace Eigen;
using namespace std;
using namespace ns3;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
    using TrajectoryFunc = std::function<Vector3d(double)>;

    const double RADIUS = 80.0;              
    const double SPEED_FACTOR = 1.0;         

    const Vector3d INITIAL_CENTER(100.0, 0.0, 50.0);

    Vector3d GetCurrentCenter(double t, double speed) {
        return INITIAL_CENTER + (Vector3d(speed, 0.0, 0.0) * t);
    }

    // ==========================================
    // TRAIETTORIE ESISTENTI (SCENARY 1)
    // ==========================================
    TrajectoryFunc MakeOctahedronFormation(int id, double speed) {
        return [id, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center; 

            double omega = 0.2 * SPEED_FACTOR; 
            double theta = omega * t;
            double local_x = 0, local_y = 0, local_z = 0;

            switch(id) { 
                case 1: local_z = RADIUS; break;
                case 2: local_z = -RADIUS; break;  
                case 3: 
                    local_x = RADIUS * cos(theta); 
                    local_y = RADIUS * sin(theta); 
                    break;
                case 4: 
                    local_x = RADIUS * cos(theta + M_PI*2/3); 
                    local_y = RADIUS * sin(theta + M_PI*2/3); 
                    break;
                case 5: 
                    local_x = RADIUS * cos(theta + M_PI*4/3); 
                    local_y = RADIUS * sin(theta + M_PI*4/3); 
                    break;
            }
            return Vector3d(center.x() + local_x, center.y() + local_y, center.z() + local_z);
        };
    }
    
    TrajectoryFunc MakeFibonacciSphereFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;

            int n = id - 1;
            int N = total - 1;

            double phi = acos(1.0 - 2.0 * (n + 0.5) / N); 
            double goldenRatio = (1.0 + sqrt(5.0)) / 2.0;
            double theta = 2.0 * M_PI * n / goldenRatio;  

            double rotationOmega = 0.2 * SPEED_FACTOR;
            theta += rotationOmega * t;

            double local_x = RADIUS * sin(phi) * cos(theta);
            double local_y = RADIUS * sin(phi) * sin(theta);
            double local_z = RADIUS * cos(phi);

            return Vector3d(center.x() + local_x, center.y() + local_y, center.z() + local_z);
        };
    }

    // ==========================================
    // NUOVE TRAIETTORIE
    // ==========================================

    // SCENARY 2: PESSIMA (Caotica e geometricamente svantaggiosa)
    TrajectoryFunc MakePessimaFormation(int id, double speed) {
        return [id, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;
            
            // Movimento basato su curve di Lissajous con frequenze sballate
            double local_x = RADIUS * sin(0.3 * t + id) * cos(0.1 * t);
            double local_y = RADIUS * cos(0.4 * t - id) * sin(0.2 * t);
            double local_z = RADIUS * sin(0.5 * t * id) * 0.5; // Z molto schiacciata (pessimo per il 3D)
            
            return Vector3d(center.x() + local_x, center.y() + local_y, center.z() + local_z);
        };
    }

    // SCENARY 3: MEDIA (Anello cilindrico sfalsato)
    TrajectoryFunc MakeMediaFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;

            // Orbita circolare piana, sfalsata su Z in base a pari/dispari
            double theta = (2.0 * M_PI * id / (total - 1)) + (0.2 * t * SPEED_FACTOR);
            double local_x = RADIUS * cos(theta);
            double local_y = RADIUS * sin(theta);
            double local_z = (id % 2 == 0) ? 20.0 : -20.0; 

            return Vector3d(center.x() + local_x, center.y() + local_y, center.z() + local_z);
        };
    }

    // SCENARY 4: IDEALE (Sfera perfetta che ruota su assi multipli)
    TrajectoryFunc MakeIdealeFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;

            int n = id - 1;
            int N = total - 1;

            double phi = acos(1.0 - 2.0 * (n + 0.5) / N); 
            double theta = 2.0 * M_PI * n / ((1.0 + sqrt(5.0)) / 2.0);  

            // Rotazione dinamica su due assi per variare costantemente la prospettiva
            theta += 0.3 * t * SPEED_FACTOR; 
            phi += sin(0.1 * t) * 0.5; // Lieve oscillazione polare

            double local_x = RADIUS * sin(phi) * cos(theta);
            double local_y = RADIUS * sin(phi) * sin(theta);
            double local_z = RADIUS * cos(phi);

            return Vector3d(center.x() + local_x, center.y() + local_y, center.z() + local_z);
        };
    }

    // SCENARY 5: REALISTICA (Tutti dritti in volo, poi accerchiamento edificio)
    // SCENARY 5: REALISTICA (Ottimizzata per sciami numerosi, 60+ droni)
    TrajectoryFunc MakeRealisticaFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            // Setup dell'edificio da circondare
            Vector3d buildingCenter(300.0, 0.0, 50.0); 
            double buildingRadius = 55.0; // Raggio aumentato per ospitare 60 droni
            
            Vector3d swarmCenter = GetCurrentCenter(t, speed);

            // 1. Calcolo Posizione V-Formation (Sciame ad alta densità)
            // Ridotti i moltiplicatori per compattare lo stormo
            double offset_x = -1.2 * id; 
            double offset_y = (id % 2 == 0 ? 1 : -1) * 1.5 * id;
            // Aggiunto sfasamento su 4 livelli di altezza per evitare collisioni fisiche nella V
            double offset_z = (id % 4) * 2.5; 
            
            Vector3d pos_V = swarmCenter + Vector3d(offset_x, offset_y, offset_z);

            // 2. Calcolo Posizione Orbita (Accerchiamento)
            double t_reach = (buildingCenter.x() - INITIAL_CENTER.x()) / speed; 
            double timeOrbiting = t - t_reach;
            
            double omega = 0.05; 
            double theta = (2.0 * M_PI * id / total) + (omega * timeOrbiting);
            // Sfasamento verticale morbido, distribuito meglio per tanti droni (id * 0.5)
            double altitudine = 50.0 + 15.0 * sin(timeOrbiting * 0.05 + id * 0.5);

            Vector3d pos_Orbit(
                buildingCenter.x() + buildingRadius * cos(theta),
                buildingCenter.y() + buildingRadius * sin(theta),
                altitudine
            );

            // 3. Logica di Blending / Transizione
            // Tempo aumentato a 20s per permettere ai droni di coda di curvare senza accelerazioni impossibili
            double transition_duration = 45.0; 
            double t_start_transition = t_reach - transition_duration;

            if (t <= t_start_transition) {
                // FASE 1: Volo in formazione compatta
                return pos_V;
            } 
            else if (t < t_reach) {
                // FASE 2: Transizione fluida (manovra di allargamento)
                double progress = (t - t_start_transition) / transition_duration;
                // Funzione Smoothstep
                //double alpha = progress * progress * (3.0 - 2.0 * progress); 
                double alpha = progress * progress * progress * (progress * (progress * 6.0 - 15.0) + 10.0);
                //Ken Perlin creatore degli effetti speciali di Tron

                return pos_V * (1.0 - alpha) + pos_Orbit * alpha;
            } 
            else {
                // FASE 3: Orbita a 360 gradi attorno all'edificio
                return pos_Orbit;
            }
        };
    }
    // DISPATCHER
    // ==========================================

    TrajectoryFunc GetAnchorTrajectory(int id, int total, double speed, int scenary) {
        switch (scenary) {
            case 1: 
                return (total <= 6) ? MakeOctahedronFormation(id, speed) : MakeFibonacciSphereFormation(id, total, speed);
            case 2: 
                return MakePessimaFormation(id, speed);
            case 3: 
                return MakeMediaFormation(id, total, speed);
            case 4: 
                return MakeIdealeFormation(id, total, speed);
            case 5: 
                return MakeRealisticaFormation(id, total, speed);
            default: 
                return MakeFibonacciSphereFormation(id, total, speed);
        }
    }

    TrajectoryFunc GetTargetTrajectory(double speed, int scenary) {
        switch (scenary) {
            case 1: return MakeFibonacciSphereFormation(0, 1, speed); 
            case 2: return MakePessimaFormation(0, speed);
            case 3: return MakeMediaFormation(0, 1, speed);
            case 4: return MakeIdealeFormation(0, 1, speed);
            case 5: return MakeRealisticaFormation(0, 1, speed); // Nel caso 5, il target si comporta come gli altri!
            default: return MakeFibonacciSphereFormation(0, 1, speed);
        }
    }
}

void AssignTrajectoryToNode(Ptr<Node> node, int id, int total_nodes, double simTime, double speed, double step_sec, int scenary) 
{
    Ptr<WaypointMobilityModel> mob = node->GetObject<WaypointMobilityModel>();
    
    if (!mob) {
        std::cerr << "ERRORE: WaypointMobilityModel non trovato sul nodo " << id << std::endl;
        return;
    }
    
    TrajectoryFunc trajFunc;
    if (id == 0) {
        trajFunc = GetTargetTrajectory(speed, scenary);
    } else {
        trajFunc = GetAnchorTrajectory(id, total_nodes, speed, scenary);
    }

    for (double t = 0.0; t <= simTime + step_sec; t += step_sec) 
    {
        Vector3d math_pos = trajFunc(t);
        ns3::Vector ns3_pos(math_pos.x(), math_pos.y(), math_pos.z());
        mob->AddWaypoint(Waypoint(Seconds(t), ns3_pos));
    }
}
