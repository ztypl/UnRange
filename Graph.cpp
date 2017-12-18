#include "Graph.h"

#define INF INT32_MAX

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

Graph* Graph::readRoadNetwork(const char* nodeFile, const char* edgeFile)
{
    FILE* node_file = fopen(nodeFile, "r");
    FILE* edge_file = fopen(edgeFile, "r");
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

