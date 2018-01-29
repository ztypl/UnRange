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
//    HisGraph* test_g = new HisGraph(4);
//    test_g->addUndirectedEdge(0,1,2);
//    test_g->addUndirectedEdge(0,2,2);
//    test_g->addUndirectedEdge(0,3,4);
//    test_g->addUndirectedEdge(1,2,4);
//    test_g->addUndirectedEdge(1,3,2);
//    test_g->addUndirectedEdge(2,3,2);


    //auto *x = g->findAllSimplePathsByDFS(213367, 144137, 2200);

    auto* x = g->getRangeRoadSegments(204452, 1000);
    for (auto& a : *x)
    {
        cout<< "Edge " << a.edge_id << " from " << a.s << " to " << a.e <<endl;
    }

    return 0;
}