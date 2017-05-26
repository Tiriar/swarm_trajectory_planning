#ifndef SWARM_TRAJECTORY_PLANNING_GRAPH_H
#define SWARM_TRAJECTORY_PLANNING_GRAPH_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <float.h>

std::vector<int> find_path(const std::string& fpath, int flock_size, int source_idx, int goal_idx);
double evaluate_edge(std::vector<double> n1, std::vector<double> n2, int flock_size, double gap);
int minDistance(double dist[], bool sptSet[], unsigned long vnum);
std::vector<int> get_path(int parent[], int j, std::vector<int> out);
std::vector<int> dijkstra(std::vector<std::vector<double>> graph, int src, int goal, unsigned long vnum);
std::vector<std::vector<double>> load_vertices(const std::string& path);
std::vector<std::vector<int>> load_edges(const std::string& path);
std::vector<std::string> split(const std::string& s);

#endif //SWARM_TRAJECTORY_PLANNING_GRAPH_H
