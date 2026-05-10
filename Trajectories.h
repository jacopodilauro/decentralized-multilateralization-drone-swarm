#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/nstime.h"

// ===========================================================================
// AssignTrajectoryToNode
//
// Assegna waypoint ns-3 al nodo in base allo scenario e agli eventi
// di join/leave dinamici.
//
// Parametri:
//   node        — nodo ns-3 a cui assegnare i waypoint
//   id          — ID del drone (0 = target)
//   total_nodes — numero totale di nodi nella simulazione
//   simTime     — durata simulazione in secondi
//   speed       — velocità lineare dello sciame [m/s]
//   step_sec    — risoluzione waypoint in secondi (default 0.5)
//   scenary     — scenario traiettoria (1-5, default 1)
//   t_join      — tempo di ingresso nello sciame (-1 = presente dall'inizio)
//   t_leave     — tempo di uscita dallo sciame   (-1 = non esce mai)
// ===========================================================================
void AssignTrajectoryToNode(
    ns3::Ptr<ns3::Node> node,
    int                 id,
    int                 total_nodes,
    double              simTime,
    double              speed,
    double              step_sec = 0.5,
    int                 scenary  = 1
);

// ===========================================================================
// AssignGuestTrajectory
//
// Traiettoria in tre fasi per droni ospiti (join/leave dinamico):
//
//   FASE 1 — AVVICINAMENTO  [t_join-15s .. t_join]
//     Parte da fuori portata UWB (Y+ rispetto al centro sciame)
//     e si inserisce dolcemente nel suo slot Fibonacci con smoothstep.
//
//   FASE 2 — ORBITA         [t_join .. t_leave]
//     Occupa uno slot nella sfera Fibonacci sincronizzato con gli altri.
//
//   FASE 3 — ALLONTANAMENTO [t_leave .. t_leave+15s]
//     Vira verso Y+ con smoothstep mentre X segue lo sciame,
//     poi sparisce fuori portata UWB.
//
// Parametri:
//   node      — nodo ns-3 del drone ospite
//   guestId   — ID assegnato dal pool (>= nBase)
//   simTime   — durata totale simulazione [s]
//   speed     — velocità X dello sciame [m/s]
//   scenary   — scenario corrente
//   t_join    — tempo di ingresso [s]
//   t_leave   — tempo di uscita [s]  (<0 = orbita permanente fino a fine sim)
//   step_sec  — passo waypoint [s]
// ===========================================================================
void AssignGuestTrajectory(
    ns3::Ptr<ns3::Node> node,
    int                 guestId,
    double              simTime,
    double              speed,
    int                 scenary,
    double              t_join,
    double              t_leave  = -1.0,
    double              step_sec = 0.5
);

#endif // TRAJECTORIES_H