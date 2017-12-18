//
// Created by macbook on 2017/12/14.
//

#ifndef UNRANGE_HISGRAPH_H
#define UNRANGE_HISGRAPH_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <queue>
#include <utility>
#include <tuple>
#include <set>
#include <limits.h>
#include <exception>

using namespace std;

class HisGraph
{
private:
    const static int NUM_DIRES = 6;
    constexpr static double LATI_DELTA = 111023.45;
    constexpr static double LONG_DELTA = 84029.13;

    static double meterToLongtitude(double meter);
    static double meterToLatitude(double meter);
    static double longtitudeToMeter(double lon);
    static double latitudeToMeter(double lat);


    struct Edge {
        int adjvex;
        int cost;
        int next;
        vector<int> num_trajs;
        Edge(int _adjvex, int _cost, int _next): num_trajs(NUM_DIRES, 0)
        {
            cost = _cost;
            next = _next;
            adjvex = _adjvex;
        }
    };
    struct Vertex {
        int id;     // 邻接表下标
        int no;     // 编号
        double longitude;   //经度
        double latitude;    //纬度
        Vertex(){};
        Vertex(int _id): id(_id){};
        Vertex(int _id, double _lon, double _lat): id(_id), longitude(_lon), latitude(_lat){};
    };
    vector<Vertex> head;
    vector<Edge> adjTable;

public:
    HisGraph(unsigned long nSize);
    HisGraph(const vector<pair<double, double>>& nodes);
    ~HisGraph();
    void addEdge(int start, int end, int value);
    void addUndirectedEdge(int start, int end, int value);
    unsigned long getNodeNum();
    int getDistance(int s, int e);

    static HisGraph* readRoadNetwork(const char* nodeFile, const char* edgeFile);


private:
    typedef vector<Vertex*> Trajectory;
    static const int MIN_DIST = 3000;

    Edge* findEdge(Vertex* s, Vertex* e);
    int getDegree(Vertex* s);
    vector<Edge*> findAllEdges(Vertex* s);
    int getDir(Vertex* s, Vertex* e);

public:
    bool readTrajectory(const char *filename);
    void readAllTrajectories();
    void testTrajectory(const char* filename);

};

struct P {		//Dijkstra
    int id;
    int cost;
    P(int _id, int _cost = 0) :id(_id), cost(_cost) {}

    friend bool operator < (const P& a, const P& b)
    {
        return a.cost > b.cost;
    }
};

#endif //UNRANGE_HISGRAPH_H
