#ifndef SWARM_TRAJECTORY_PLANNING_GRAPH_H
#define SWARM_TRAJECTORY_PLANNING_GRAPH_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <climits>
#include <set>

struct edge {int to, length;};

int dijkstra(const std::vector<std::vector<edge>> &graph, int source, int target);
std::vector<std::vector<double>> load_vertices(const std::string& path);
std::vector<std::vector<int>> load_edges(const std::string& path);
std::vector<std::string> split(const std::string& s);

#endif //SWARM_TRAJECTORY_PLANNING_GRAPH_H
