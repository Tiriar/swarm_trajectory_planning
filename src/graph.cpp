#include "../headers/graph.h"

using namespace std;

int dijkstra(const vector<vector<edge>> &graph, int source, int target) {
    vector<int> min_distance(graph.size(), INT_MAX);
    min_distance[source] = 0;
    set<pair<int,int>> active_vertices;
    active_vertices.insert({0,source});

    while (!active_vertices.empty()) {
        int where = active_vertices.begin()->second;
        if (where == target) return min_distance[where];
        active_vertices.erase(active_vertices.begin());
        for (auto ed : graph[where])
            if (min_distance[ed.to] > min_distance[where] + ed.length) {
                active_vertices.erase({min_distance[ed.to], ed.to});
                min_distance[ed.to] = min_distance[where] + ed.length;
                active_vertices.insert({min_distance[ed.to], ed.to});
            }
    }
    return INT_MAX;
}

vector<vector<double>> load_vertices(const string& path) {
    string line;
    vector<string> temp;
    vector<double> vertex;
    vector<vector<double>> out;

    ifstream f(path);
    if (f.is_open()) {
        while (getline(f, line)) {
            if (line == "---") break;
            temp = split(line);
            vertex = {atof(temp[0].c_str()), atof(temp[1].c_str())};
            out.push_back(vertex);
        }
        f.close();
    }
    else cout << "Unable to open file" << endl;
    return out;
}

vector<vector<int>> load_edges(const string& path) {
    string line;
    vector<string> temp;
    vector<int> edge;
    vector<vector<int>> out;

    ifstream f(path);
    if (f.is_open()) {
        while (getline(f, line)) {
            if (line == "---") break;
        }
        while (getline(f, line)) {
            temp = split(line);
            edge = {atoi(temp[0].c_str()), atoi(temp[1].c_str())};
            out.push_back(edge);
        }
        f.close();
    }
    else cout << "Unable to open file" << endl;
    return out;
}

vector<string> split(const string& s) {
    vector<string> out;
    istringstream iss(s);
    do {
        string sub;
        iss >> sub;
        out.push_back(sub);
    } while (iss);
    return out;
}
