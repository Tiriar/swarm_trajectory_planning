#include "../headers/graph.h"

using namespace std;

/**
 * Main path finding function - loads graph and finds the shortest path from the start node to the end node
 * @param fpath graph file path
 * @param flock_size number of UAVs
 * @param source_idx start node
 * @param goal_idx end node
 * @return path as a vector of node indices
 */
vector<int> find_path(const string& fpath, int flock_size, int source_idx, int goal_idx) {
    vector<vector<double>> vertices = load_vertices(fpath);
    vector<vector<int>> edges = load_edges(fpath);
    unsigned long vnum = vertices.size();

    // initialize graph matrix
    vector<vector<double>> graph;
    vector<double> tmp(vnum, 0.0);
    for (int i=0; i<vnum; i++){
        graph.push_back(tmp);
    }

    // evaluate edges
    vector<int> edge;
    double eval;
    for (int i=0; i<edges.size(); i++){
        edge = edges[i];
        eval = evaluate_edge(vertices[edge[0]], vertices[edge[1]], flock_size, 1000);
        graph[edge[0]][edge[1]] = eval;
        graph[edge[1]][edge[0]] = eval;
    }

    return dijkstra(graph, source_idx, goal_idx, vnum);
}

/**
 * Evaluates graph edge
 * @param n1 node #1 coordinates
 * @param n2 node #2 coordinates
 * @param flock_size number of UAVs
 * @param gap size of gap between obstacles around the edge
 * @return edge cost
 */
double evaluate_edge(vector<double> n1, vector<double> n2, int flock_size, double gap) {
    double dist = sqrt(pow(n1[0]-n2[0], 2)+pow(n1[1]-n2[1], 2));
    if (flock_size == 1) return 1.584*dist+7.23939*exp(-0.370314*gap);
    else return 1.584*dist+(1.25827*flock_size+8.93066)*exp((0.10727*flock_size+-0.648613)*gap);
}

/**
 * A utility function to find the vertex with minimum distance value,
 * from the set of vertices not yet included in shortest path tree
 * @param dist dist[i] = the shortest distance from src to i
 * @param sptSet sptSet[i] = true if vertex i is included or in shortest path tree
 * or shortest distance from src to i is finalized
 * @param vnum number of vertices in the graph
 * @return index of closest vertex
 */
int minDistance(double dist[], bool sptSet[], unsigned long vnum) {
    double min = DBL_MAX;
    int min_index = 0;

    for (int v = 0; v < vnum; v++) {
        if (!sptSet[v] && dist[v] <= min) min = dist[v], min_index = v;
    }
    return min_index;
}

/**
 * Function to find shortest path from source to j using parent array
 * @param parent parent node array
 * @param j goal node
 */
vector<int> get_path(int parent[], int j, vector<int> out) {
    if (parent[j] == -1) return out;
    out = get_path(parent, parent[j], out);
    out.push_back(j);
    return out;
}

/**
 * Dijkstra's single source algorithm for a graph represented using adjacency matrix
 * @param graph graph representation
 * @param src source node index
 * @param vnum number of vertices in the graph
 */
vector<int> dijkstra(vector<vector<double>> graph, int src, int goal, unsigned long vnum) {
    double dist[vnum];
    bool sptSet[vnum];
    int parent[vnum];

    // initialization
    for (int i = 0; i < vnum; i++) {
        parent[0] = -1;
        dist[i] = DBL_MAX;
        sptSet[i] = false;
    }
    dist[src] = 0;

    for (int count = 0; count < vnum-1; count++) {
        int u = minDistance(dist, sptSet, vnum);
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v = 0; v < vnum; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v]) {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    vector<int> out = {};
    return get_path(parent, goal, out);
}

/**
 * Loads vertices from the given file (+converts mm to m)
 * @param path file path
 * @return vertices coordinates
 */
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
            vertex = {atof(temp[0].c_str())/1000, atof(temp[1].c_str())/1000};
            out.push_back(vertex);
        }
        f.close();
    }
    else cout << "Unable to open file" << endl;
    return out;
}

/**
 * Loads edges from the given file
 * @param path file path
 * @return edges as vertex indices
 */
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

/**
 * Utility function for string splitting using spaces as delimiter
 * @param s input string
 * @return vector of words
 */
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
