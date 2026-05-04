#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

/**
 * @brief Pre-calcola e assegna i Waypoint a un nodo ns-3 per tutta la simulazione
 * * @param node Il nodo ns-3 a cui applicare la mobilità
 * @param id L'ID del drone (0 è il target, 1+ sono gli anchor)
 * @param total_nodes Numero totale di droni nello sciame
 * @param simTime Tempo totale della simulazione
 * @param step_sec Intervallo di tempo tra un Waypoint e l'altro
 */
void AssignTrajectoryToNode(Ptr<Node> node, int id, int total_nodes, double simTime, double speed, double step_sec = 0.5, int scenary = 1);

#endif // TRAJECTORIES_H
