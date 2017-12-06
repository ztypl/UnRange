#include "Graph.h"

int INF = 0x7fffffff;

Graph::Graph(unsigned long nSize) :head(nSize, Vertex(-1))
{
}

Graph::Graph(const vector<pair<double, double>>& nodes) :head(nodes.size(), Vertex(-1))
{
    for(int i=0; i<nodes.size(); i++)
    {
        head[i].longitude = nodes[i].first;
        head[i].latitude = nodes[i].second;
    }
}

Graph::~Graph()
{
}

void Graph::addEdge(int start, int end, int value)
{
	adjTable.push_back(Edge(end, value, head[start].id));
	head[start].id = static_cast<int>(adjTable.size() - 1);
}

void Graph::addUndirectedEdge(int start, int end, int value)
{
	addEdge(start, end, value);
	addEdge(end, start, value);
}

unsigned long Graph::getNodeNum()
{
	return head.size();
}

int Graph::getDistance(int s, int e)	//Dijkstra
{
	int distance = 0;
	priority_queue<P> nextVertexQueue;
	vector<int> dis(getNodeNum(), INF);
	
	nextVertexQueue.push(P(s));
	while (!nextVertexQueue.empty())
	{
		P curVer = nextVertexQueue.top();
		nextVertexQueue.pop();

		if (dis[curVer.id] < curVer.cost)
			continue;

		dis[curVer.id] = curVer.cost;

		if (curVer.id == e)
			break;

		int i = head[curVer.id].id;
		while (i != -1)
		{
			Edge e = adjTable[i];
			if (dis[e.adjvex] > dis[curVer.id] + e.cost)
			{
				dis[e.adjvex] = dis[curVer.id] + e.cost;
				nextVertexQueue.push(P(e.adjvex, dis[e.adjvex]));
			}
			i = e.next;
		}
	}
	return dis[e];
}

