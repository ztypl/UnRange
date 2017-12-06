#include <iostream>
#include <cstdio>
#include <vector>
#include <utility>

#include "Graph.h"

using namespace std;

const char* EDGE_FILE = "../data/BJ.nedge";
const char* NODE_FILE = "../data/BJ.cnode";

Graph* readRoadNetwork()
{
    FILE* node_file = fopen(NODE_FILE, "r");
    FILE* edge_file = fopen(EDGE_FILE, "r");
    vector<pair<double, double>> nodes;
    int id;

    // reading nodes
    double a,b;
    while(~fscanf(node_file, "%d%lf%lf", &id, &a, &b))
    {
        nodes.push_back(make_pair(a,b));
    }
    Graph* g = new Graph(nodes);

    // reading edges
    int nodes_size, edges_size;
    fscanf(edge_file, "%d%d", &nodes_size, &edges_size);
    int x, y, len;
    while(~fscanf(edge_file, "%d%d%d", &x, &y, &len))
    {
        g->addUndirectedEdge(x, y, len);
    }

    fclose(node_file);
    fclose(edge_file);

    return g;
}

int main()
{
    Graph* g = readRoadNetwork();

    return 0;
}