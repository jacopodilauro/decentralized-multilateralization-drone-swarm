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
void AssignTrajectoryToNode( ns3::Ptr<ns3::Node> node, int id, int total_nodes, double simTime, double speed, double step_sec = 0.5, int scenary = 1);

void AssignGuestTrajectory( ns3::Ptr<ns3::Node> node, int guestId, double simTime, double speed, int scenary, double t_join, double t_leave  = -1.0, double step_sec = 0.5);

#endif