#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <queue>
#include <utility>
#include <tuple>
#include <set>
#include <limits.h>

using namespace std;

class Graph
{
protected:
	struct Edge {
		int adjvex;
		int cost;
		int next;
		Edge(int _adjvex, int _cost, int _next)
		{
			cost = _cost;
			next = _next;
			adjvex = _adjvex;
		}
	};
	struct Vertex {
		int id;
        double longitude;   //经度
        double latitude;    //纬度
		Vertex(int _id): id(_id){};
        Vertex(int _id, double _lon, double _lat): id(_id), longitude(_lon), latitude(_lat){};
	};
	vector<Vertex> head;
	vector<Edge> adjTable;
	
public:
	Graph(unsigned long nSize);
    Graph(const vector<pair<double, double>>& nodes);
	~Graph();
	void addEdge(int start, int end, int value);
	void addUndirectedEdge(int start, int end, int value);
	unsigned long getNodeNum();
	int getDistance(int s, int e);

	static Graph* readRoadNetwork(const char* nodeFile, const char* edgeFile);
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


#endif