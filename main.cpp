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
    HisGraph* g = HisGraph::readRoadNetwork(NODE_FILE, EDGE_FILE);
    g->readAllTrajectories();
    g->save("../data/history.ser");
//    HisGraph* g = new HisGraph("history.ser");
//    g->testTrajectory("../data/0502/ST_1.txt");
    return 0;
}