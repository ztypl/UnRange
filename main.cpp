#include <iostream>
#include <cstdio>
#include <vector>
#include <utility>

#include "HisGraph.h"

using namespace std;

const char* EDGE_FILE = "../data/BJ.nedge";
const char* NODE_FILE = "../data/BJ.cnode";
const char* DATA_DIR = "../data";

int main()
{
//    HisGraph* g = HisGraph::readRoadNetwork(NODE_FILE, EDGE_FILE);
//    g->readAllTrajectories();
//    g->save("../data/history.ser");
    HisGraph *g = new HisGraph("../data/history.ser");

    auto *x = g->findAllPathByBFS(213367, 274474, 1000);
    for (auto *p : *x)
    {
        for (auto i : *p)
        {
            printf("%d,", i);
        }
        puts("");
    }
//    cout << g->getDistance(213367, 274474) << endl;
    return 0;
}