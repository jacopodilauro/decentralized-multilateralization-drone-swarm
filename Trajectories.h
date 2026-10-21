#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/nstime.h"

// Fattore di scala della formazione (strada A): tutte le distanze x S, le velocita' angolari / S, quindi le
// velocita' lineari restano identiche. S = 1: traiettorie storiche, identiche bit per bit
void SetFormationScale(double s);
double GetFormationScale();
// Geometria dei droni base (paper): 0 = storica (default, identica), 1 = guscio sferico a spaziatura costante:
// nodo 0 fermo al centro, gli altri N-1 su una sfera (punti di Fibonacci, asse verticale) di raggio scelto perche'
// la distanza mediana dal vicino piu' prossimo valga `spacing` (minimo `spacing`), rotazione rigida attorno alla
// verticale a `speed` m/s all'equatore. Confrontabile al variare di N: stessa spaziatura, stessa velocita' lineare
void SetFormation(int type, double spacing, double speed);
int GetFormation();
double ShellRadius(int n);   // raggio del guscio con n droni sul guscio (n = N - 1)
void AssignTrajectoryToNode( ns3::Ptr<ns3::Node> node, int id, int total_nodes, double simTime, double speed, double step_sec = 0.5, int scenary = 1);

void AssignGuestTrajectory( ns3::Ptr<ns3::Node> node, int guestId, double simTime, double speed, int scenary, double t_join, double t_leave  = -1.0, double step_sec = 0.5);

#endif