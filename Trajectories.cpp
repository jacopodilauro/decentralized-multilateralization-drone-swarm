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

    constexpr double RADIUS       = 80.0;
    const double SPEED_FACTOR = 1.0;
    const Vector3d INITIAL_CENTER(100.0, 0.0, 50.0);

    Vector3d GetCurrentCenter(double t, double speed) {
        return INITIAL_CENTER + (Vector3d(speed, 0.0, 0.0) * t);
    }

    // -----------------------------------------------------------------------
    // Ken Perlin
    // -----------------------------------------------------------------------
    double Smoothstep(double x) {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
    }

    // ==========================================
    // TRAIETTORIE BASE
    // ==========================================

    TrajectoryFunc MakeOctahedronFormation(int id, double speed) {
        return [id, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;
            double omega = 0.2 * SPEED_FACTOR;
            double theta = omega * t;
            double lx = 0, ly = 0, lz = 0;
            switch(id) {
                case 1: lz =  RADIUS; break;
                case 2: lz = -RADIUS; break;
                case 3: lx = RADIUS*cos(theta);            ly = RADIUS*sin(theta);            break;
                case 4: lx = RADIUS*cos(theta+M_PI*2/3);  ly = RADIUS*sin(theta+M_PI*2/3);  break;
                case 5: lx = RADIUS*cos(theta+M_PI*4/3);  ly = RADIUS*sin(theta+M_PI*4/3);  break;
            }
            return Vector3d(center.x()+lx, center.y()+ly, center.z()+lz);
        };
    }

    TrajectoryFunc MakeFibonacciSphereFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;
            int n = id - 1;
            int N = total - 1;
            double phi   = acos(1.0 - 2.0*(n+0.5)/N);
            double theta = 2.0*M_PI*n / ((1.0+sqrt(5.0))/2.0);
            theta += 0.2 * SPEED_FACTOR * t;
            return Vector3d(
                center.x() + RADIUS*sin(phi)*cos(theta),
                center.y() + RADIUS*sin(phi)*sin(theta),
                center.z() + RADIUS*cos(phi));
        };
    }

    TrajectoryFunc MakePessimaFormation(int id, double speed) {
        return [id, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;
            return Vector3d(
                center.x() + RADIUS*sin(0.3*t+id)*cos(0.1*t),
                center.y() + RADIUS*cos(0.4*t-id)*sin(0.2*t),
                center.z() + RADIUS*sin(0.5*t*id)*0.5);
        };
    }

    TrajectoryFunc MakeMediaFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;
            double theta = (2.0*M_PI*id/(total-1)) + (0.2*t*SPEED_FACTOR);
            return Vector3d(
                center.x() + RADIUS*cos(theta),
                center.y() + RADIUS*sin(theta),
                center.z() + ((id%2==0) ? 20.0 : -20.0));
        };
    }

    TrajectoryFunc MakeIdealeFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d center = GetCurrentCenter(t, speed);
            if (id == 0) return center;
            int n = id-1, N = total-1;
            double phi   = acos(1.0 - 2.0*(n+0.5)/N);
            double theta = 2.0*M_PI*n / ((1.0+sqrt(5.0))/2.0);
            theta += 0.3*t*SPEED_FACTOR;
            phi   += sin(0.1*t)*0.5;
            return Vector3d(
                center.x() + RADIUS*sin(phi)*cos(theta),
                center.y() + RADIUS*sin(phi)*sin(theta),
                center.z() + RADIUS*cos(phi));
        };
    }

    TrajectoryFunc MakeRealisticaFormation(int id, int total, double speed) {
        return [id, total, speed](double t) -> Vector3d {
            Vector3d buildingCenter(300.0, 0.0, 50.0);
            double   buildingRadius = 55.0;
            Vector3d swarmCenter    = GetCurrentCenter(t, speed);

            Vector3d pos_V = swarmCenter + Vector3d(
                -1.2*id,
                (id%2==0?1:-1)*1.5*id,
                (id%4)*2.5);

            double t_reach      = (buildingCenter.x()-INITIAL_CENTER.x()) / speed;
            double timeOrbit    = t - t_reach;
            double theta        = (2.0*M_PI*id/total) + (0.05*timeOrbit);
            double alt          = 50.0 + 15.0*sin(timeOrbit*0.05 + id*0.5);

            Vector3d pos_Orbit(
                buildingCenter.x() + buildingRadius*cos(theta),
                buildingCenter.y() + buildingRadius*sin(theta),
                alt);

            double t_trans = t_reach - 45.0;
            if      (t <= t_trans) return pos_V;
            else if (t <  t_reach) {
                double a = Smoothstep((t-t_trans)/45.0);
                return pos_V*(1.0-a) + pos_Orbit*a;
            }
            else return pos_Orbit;
        };
    }

    // ==========================================
    // TRAIETTORIA OSPITE
    // ==========================================

    // Durate manovre [s]
    constexpr double APPROACH_DURATION = 15.0;
    constexpr double DEPART_DURATION   = 15.0;

    // Distanza di partenza dall'orbita [m] — fuori portata UWB all'inizio
    constexpr double ENTRY_DISTANCE = RADIUS * 3.5;   // ~280m

    // ------------------------------------------------------------------
    // Slot Fibonacci che spetta all'ospite.
    // Usiamo totalBase fisso per non scompaginare la sfera principale.
    // L'ID ospite viene mappato con modulo così è sempre ben distribuito.
    // ------------------------------------------------------------------
    Vector3d FibonacciSlot(int guestId, double t, double speed, int totalBase) {
        Vector3d center = GetCurrentCenter(t, speed);
        int n = guestId % std::max(1, totalBase);
        int N = std::max(2, totalBase);
        double phi   = acos(1.0 - 2.0*(n+0.5)/N);
        double theta = 2.0*M_PI*n / ((1.0+sqrt(5.0))/2.0);
        theta += 0.2 * SPEED_FACTOR * t;   // sincronizzato con gli altri
        return Vector3d(
            center.x() + RADIUS*sin(phi)*cos(theta),
            center.y() + RADIUS*sin(phi)*sin(theta),
            center.z() + RADIUS*cos(phi));
    }

    // ------------------------------------------------------------------
    // Punto di partenza dell'avvicinamento:
    // Y+ rispetto al centro sciame al momento del join, quota invariata.
    // Rimane fisso nello spazio (il drone deve rincorrere l'orbita).
    // ------------------------------------------------------------------
    Vector3d EntryPos(double t_join, double speed) {
        Vector3d center = GetCurrentCenter(t_join, speed);
        return Vector3d(center.x(), center.y() + ENTRY_DISTANCE, center.z());
    }

    // ------------------------------------------------------------------
    // Traiettoria dopo il leave:
    //   - X: segue lo speed dello sciame (rimane nel frame orizzontale)
    //   - Y: si allontana verso Y+ a velocità costante
    //   - Z: quota al momento del leave (costante)
    // ------------------------------------------------------------------
    Vector3d DepartPos(int guestId, double t_leave, double t_now,
                       double speed, int totalBase)
    {
        Vector3d orbitAtLeave = FibonacciSlot(guestId, t_leave, speed, totalBase);
        double dt             = t_now - t_leave;
        double depart_vy      = ENTRY_DISTANCE / DEPART_DURATION;   // [m/s]
        return Vector3d(
            orbitAtLeave.x() + speed * dt,      // segue X dello sciame
            orbitAtLeave.y() + depart_vy * dt,  // vira verso fuori
            orbitAtLeave.z());                   // quota costante
    }

    // ------------------------------------------------------------------
    // MakeGuestTrajectory — assembla le tre fasi con smoothstep
    // ------------------------------------------------------------------
    TrajectoryFunc MakeGuestTrajectory(int guestId, double t_join,
                                       double t_leave, double speed,
                                       int totalBase)
    {
        return [=](double t) -> Vector3d
        {
            const double t_approach_start = t_join - APPROACH_DURATION;

            // Prima dell'avvicinamento: fermo all'entry position
            if (t < t_approach_start) {
                return EntryPos(t_join, speed);
            }

            // ---- FASE 1: AVVICINAMENTO ----
            if (t < t_join) {
                double alpha      = Smoothstep((t - t_approach_start) / APPROACH_DURATION);
                Vector3d startPos = EntryPos(t_join, speed);
                Vector3d slotPos  = FibonacciSlot(guestId, t, speed, totalBase);
                return startPos*(1.0-alpha) + slotPos*alpha;
            }

            // ---- FASE 2: ORBITA ----
            bool   hasLeave      = (t_leave > 0.0);
            double t_depart_end  = hasLeave ? (t_leave + DEPART_DURATION) : 1e9;
            double t_depart_start = hasLeave ? t_leave : 1e9;

            if (t < t_depart_start) {
                return FibonacciSlot(guestId, t, speed, totalBase);
            }

            // ---- FASE 3: ALLONTANAMENTO ----
            if (t < t_depart_end) {
                double alpha      = Smoothstep((t - t_depart_start) / DEPART_DURATION);
                Vector3d orbitPos = FibonacciSlot(guestId, t_depart_start, speed, totalBase);
                Vector3d departPos = DepartPos(guestId, t_depart_start, t, speed, totalBase);
                return orbitPos*(1.0-alpha) + departPos*alpha;
            }

            // Dopo l'allontanamento: continua dritto fuori portata
            return DepartPos(guestId, t_depart_start,
                             t_depart_end, speed, totalBase)
                   + Vector3d(speed*(t - t_depart_end), 0.0, 0.0);
        };
    }

    // ==========================================
    // DISPATCHER
    // ==========================================

    TrajectoryFunc GetAnchorTrajectory(int id, int total, double speed, int scenary) {
        switch (scenary) {
            case 1:  return (total <= 6) ? MakeOctahedronFormation(id, speed)
                                         : MakeFibonacciSphereFormation(id, total, speed);
            case 2:  return MakePessimaFormation(id, speed);
            case 3:  return MakeMediaFormation(id, total, speed);
            case 4:  return MakeIdealeFormation(id, total, speed);
            case 5:  return MakeRealisticaFormation(id, total, speed);
            default: return MakeFibonacciSphereFormation(id, total, speed);
        }
    }

    TrajectoryFunc GetTargetTrajectory(double speed, int scenary) {
        switch (scenary) {
            case 1:  return MakeFibonacciSphereFormation(0, 1, speed);
            case 2:  return MakePessimaFormation(0, speed);
            case 3:  return MakeMediaFormation(0, 1, speed);
            case 4:  return MakeIdealeFormation(0, 1, speed);
            case 5:  return MakeRealisticaFormation(0, 1, speed);
            default: return MakeFibonacciSphereFormation(0, 1, speed);
        }
    }

} // namespace anonimo

// ===========================================================================
// AssignTrajectoryToNode — invariata
// ===========================================================================
void AssignTrajectoryToNode(Ptr<Node> node, int id, int total_nodes,
                             double simTime, double speed,
                             double step_sec, int scenary)
{
    Ptr<WaypointMobilityModel> mob = node->GetObject<WaypointMobilityModel>();
    if (!mob) {
        std::cerr << "ERRORE: WaypointMobilityModel non trovato sul nodo "
                  << id << std::endl;
        return;
    }

    TrajectoryFunc trajFunc = (id == 0)
        ? GetTargetTrajectory(speed, scenary)
        : GetAnchorTrajectory(id, total_nodes, speed, scenary);

    for (double t = 0.0; t <= simTime + step_sec; t += step_sec) {
        Vector3d    p = trajFunc(t);
        mob->AddWaypoint(Waypoint(Seconds(t), ns3::Vector(p.x(), p.y(), p.z())));
    }
}

// ===========================================================================
// AssignGuestTrajectory — NUOVA (Passo 2)
//
// Chiamata da OnDroneJoin in main.cpp al momento del join.
// Sovrascrive i waypoint di "parcheggio" (-9999) con la traiettoria reale.
//
// WaypointMobilityModel accetta waypoint anche passati rispetto al tempo
// corrente del simulatore se il nodo non si è ancora mosso da lì —
// ns-3 interpola linearmente tra l'ultimo waypoint valido e il prossimo.
// Aggiungere waypoint da t=0 è quindi sicuro e garantisce che il modello
// abbia sempre una posizione definita.
// ===========================================================================
void AssignGuestTrajectory(Ptr<Node> node, int guestId,
                            double simTime, double speed,
                            int scenary,
                            double t_join, double t_leave,
                            double step_sec)
{
    Ptr<WaypointMobilityModel> mob = node->GetObject<WaypointMobilityModel>();
    if (!mob) {
        std::cerr << "ERRORE: WaypointMobilityModel non trovato sul nodo ospite "
                  << guestId << std::endl;
        return;
    }

    // totalBase = 8 per Scenary 1 (Fibonacci).
    // Per altri scenari l'ospite usa comunque Fibonacci come orbita
    // di inserimento — è la scelta più robusta geometricamente.
    const int totalBase = 8;

    TrajectoryFunc guestFunc = MakeGuestTrajectory(
        guestId, t_join, t_leave, speed, totalBase);

    // Aggiungiamo waypoint da t=0 così ns-3 ha sempre una posizione valida.
    // Quelli prima di (t_join - APPROACH_DURATION) puntano all'entry position,
    // quindi il drone risulta fermo fuori portata fino all'avvicinamento.
    for (double t = 0.0; t <= simTime + step_sec; t += step_sec) {
        Vector3d    p = guestFunc(t);
        mob->AddWaypoint(Waypoint(Seconds(t), ns3::Vector(p.x(), p.y(), p.z())));
    }

    std::cout << "[GuestTraj] ID=" << guestId
              << " | avvicinamento @ t=" << (t_join - APPROACH_DURATION) << "s"
              << " | orbita @ t="        << t_join                        << "s"
              << (t_leave > 0
                  ? " | allontanamento @ t=" + std::to_string(int(t_leave)) + "s"
                  : " | orbita permanente")
              << std::endl;
}