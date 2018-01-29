//
// Created by macbook on 2017/12/14.
//

#include "HisGraph.h"

#define INF INT32_MAX
#define PI M_PI

HisGraph::HisGraph(unsigned long nSize) :head(nSize, Vertex(-1))
{
    for(int i=0; i<head.size(); i++)
    {
        head[i].no = i;
    }
}

HisGraph::HisGraph(const vector<pair<double, double>>& nodes) :head(nodes.size(), Vertex(-1))
{
    for(int i=0; i<head.size(); i++)
    {
        head[i].no = i;
        head[i].longitude = nodes[i].first;
        head[i].latitude = nodes[i].second;
    }
}

HisGraph::~HisGraph()
{
}

void HisGraph::addEdge(int start, int end, int value)
{
    adjTable.push_back(Edge(end, value, head[start].id));
    head[start].id = static_cast<int>(adjTable.size() - 1);
}

void HisGraph::addUndirectedEdge(int start, int end, int value)
{
    addEdge(start, end, value);
    addEdge(end, start, value);
}

unsigned long HisGraph::getNodeNum()
{
    return head.size();
}

int HisGraph::getDistance(int s, int e)	//Dijkstra
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

HisGraph* HisGraph::readRoadNetwork(const char* nodeFile, const char* edgeFile)
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
    HisGraph* g = new HisGraph(nodes);

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

double HisGraph::meterToLongtitude(double meter)
{
    return meter / LONG_DELTA;
}

double HisGraph::meterToLatitude(double meter)
{
    return meter / LATI_DELTA;
}

double HisGraph::longtitudeToMeter(double lon)
{
    return lon * LONG_DELTA;
}

double HisGraph::latitudeToMeter(double lat)
{
    return lat * LATI_DELTA;
}


HisGraph::Edge* HisGraph::findEdge(Vertex *s, Vertex *e)
{
    int e_id = s->id;
    while (e_id != -1)
    {
        if (adjTable[e_id].adjvex == e->no)
            return &(adjTable[e_id]);
        else
            e_id = adjTable[e_id].next;
    }
    return NULL;
}

int HisGraph::getDegree(Vertex* s)
{
    int n = 0;
    int e_id = s->id;
    while (e_id != -1)
    {
        n++;
        e_id = adjTable[e_id].next;
    }
    return n;
}

vector<HisGraph::Edge*> HisGraph::findAllEdges(Vertex* s)
{
    vector<Edge*> ve;
    int e_id = s->id;
    while (e_id != -1)
    {
        ve.push_back(&(adjTable[e_id]));
        e_id = adjTable[e_id].next;
    }
    return ve;
}

int HisGraph::getDir(Vertex* s, Vertex* e)
{
    double x = longtitudeToMeter(e->longitude - s->longitude);
    double y = latitudeToMeter(e->latitude - s->latitude);

    double alpha = atan(y / x);
    if (x < 0)
        alpha += PI;
    if (alpha < 0)
        alpha += 2 * PI;

    if (alpha == alpha) // alpha is not NaN
        return static_cast<int>(floor(alpha * NUM_DIRES / 2 / PI));
    else return -1;
}

bool HisGraph::readTrajectory(const char *filename)
{
    FILE* f = fopen(filename, "r");
    if (f == NULL)
        return false;
    Trajectory* traj = new Trajectory();
    char str[100];
    // int h, m, s;
    while(~fscanf(f, "%s", str))
    {
        int node;
        if (str[2]!=':')
        {
            node = atoi(str);
        }
        else
        {
            //sscanf(str, "%d:%d:%d %d", &h, &m, &s, &node);
            continue;
        }
        if (traj->size() == 0 or node != traj->back()->no)
            traj->push_back(&(head[node]));
    }

    int dist = 0;
    int i = 0, j = i;

    for (j = 1; dist < MIN_DIST && j < traj->size()-1; j++)
    {
        int s = traj->at(j)->no;
        int e = traj->at(j+1)->no;
        dist += getDistance(s, e);
    }

    Edge* e = NULL;
    while (i + 6 < traj->size())
    {
        double x = longtitudeToMeter(traj->at(j)->longitude - traj->at(i)->longitude);
        double y = latitudeToMeter(traj->at(j)->latitude - traj->at(i)->latitude);

        double alpha = atan(y / x);
        if (x < 0)
            alpha += PI;
        if (alpha < 0)
            alpha += 2 * PI;

        if (alpha == alpha) // alpha is not NaN
        {
            int dir = static_cast<int>(floor(alpha * NUM_DIRES / 2 / PI));

            e = findEdge(traj->at(i), traj->at(i + 1));
            if (e == NULL)
            {
                cerr << "No Edge Between Vertex " << traj->at(i)->no << " and " << traj->at(i + 1)->no << "." << endl;
            }
            else
            {
                e->num_trajs[dir]++;
//                cout << "Edge Between Vertex " << traj->at(i)->no << " and " << traj->at(i + 1)->no
//                     << " dir " << dir << " +1." << endl;
            }
        }
        // 下一个点
        i++;
        dist -= e ? e->cost: 0;
        for(; dist < MIN_DIST && j < traj->size()-1; j++)
        {
            int s = traj->at(j)->no;
            int e = traj->at(j+1)->no;
            dist += getDistance(s, e);
        }
    }
    delete traj;
    fclose(f);
    cout << "Read File " <<filename <<" Done." <<endl;
    return true;
}

void HisGraph::readAllTrajectories()
{
    int N = 8;
    char filename[100];
    for (int i=1; i<=N; i++)
    {
        for (int j=1; ;j++)
        {
            sprintf(filename, "../data/05%02d/ST_%d.txt", i, j);

            if (!readTrajectory(filename))
            {
                cout << "Dir " << i <<" Done." <<endl;
                break;
            }
        }
    }

}

void HisGraph::testTrajectory(const char* filename)
{
    FILE* f = fopen(filename, "r");
    if (f == NULL)
    {
        puts("Test file not exists.");
        return;
    }
    Trajectory* traj = new Trajectory();
    vector<int> sample_id;
    char str[100];
    int h, m, s;
    int count = 3;
    while(~fscanf(f, "%s", str))
    {
        int node;
        if (str[2]!=':')
        {
            node = atoi(str);
        }
        else
        {
            count++;
            continue;
        }
        if (traj->size() == 0 or node != traj->back()->no)
        {
            traj->push_back(&(head[node]));
            if (count == 4)
            {
                count = 0;
                sample_id.push_back(traj->size()-1);
            }
        }
    }

    for (int i=0; i<sample_id.size()-1; i++)
    {
        int p = sample_id[i];
        int q = sample_id[i+1];
        Vertex* d = traj->at(q);

        long double allpro1 = 1.0;
        long double allpro2 = 1.0;

        for(;p<q;p++)
        {
            Vertex* v = traj->at(p);
            Vertex* n = traj->at(p+1);

            int dir = getDir(v, d);
            if (dir == -1)
            {
                cerr << "Error between node " << v->no <<" and " << d->no <<endl;
                continue;
            }
            vector<Edge*> eg = findAllEdges(v);

            long double pro2 = 1.0 / eg.size();

            int total_num = 0, num = 0;
            for(auto e : eg)
            {
                total_num += e->num_trajs[dir];
                if (e->adjvex == n->no)
                    num = e->num_trajs[dir];
            }

            long double pro1;
            if (total_num < 4) //TODO: change to const support
                pro1 = pro2;
            else
                pro1 = (long double)(num) / (long double)(total_num);

            allpro1 *= pro1;
            allpro2 *= pro2;
        }
        cout << "Prob1: " << allpro1 <<"\tProb2: " << allpro2 <<endl;
    }
    fclose(f);
    delete traj;
}


vector<vector<int>*>* HisGraph::findAllPathsByDFS(int a, int b, int MAX_DIST)
{
    vector<int> stack;
    vector<int> stack_e;
    int dist = 0;       // 记录栈中路径的距离
    stack.push_back(a);
    stack_e.push_back(head[a].id);
    vector<vector<int>*>* result = new vector<vector<int>*>;
    while (!stack.empty())
    {
        int cur_node = stack.back();
        int eid = stack_e.back();
        if (dist <= MAX_DIST)
        {
            if (cur_node == b)
            {
                result->push_back(new vector<int>(stack));
                stack.pop_back();
                stack_e.pop_back();
                dist -= adjTable[stack_e.back()].cost;
                stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
            }
            else
            {
                if (eid == -1)
                {
                    stack.pop_back();
                    stack_e.pop_back();
                    dist -= adjTable[stack_e.back()].cost;
                    if (!stack_e.empty())
                        stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
                }
                else
                {
                    stack_e[stack_e.size()-1] = eid;
                    dist += adjTable[eid].cost;
                    int node = adjTable[eid].adjvex;
                    stack.push_back(node);
                    stack_e.push_back(head[node].id);
                }
            }
        }
        else
        {
            stack.pop_back();
            stack_e.pop_back();
            dist -= adjTable[stack_e.back()].cost;
            stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
        }
    }
    return result;
}

vector<vector<int>*>* HisGraph::findAllSimplePathsByDFS(int a, int b, int MAX_DIST)
{
    vector<int> stack;
    vector<int> stack_e;
    int dist = 0;       // 记录栈中路径的距离
    stack.push_back(a);
    stack_e.push_back(head[a].id);
    vector<bool> in_stack(this->head.size(), false);
    in_stack[a] = true;
    vector<vector<int>*>* result = new vector<vector<int>*>;
    while (!stack.empty())
    {
        int cur_node = stack.back();
        int eid = stack_e.back();
        if (dist <= MAX_DIST)
        {
            if (cur_node == b)
            {
                result->push_back(new vector<int>(stack));
                in_stack[stack.back()] = false;
                stack.pop_back();
                stack_e.pop_back();
                dist -= adjTable[stack_e.back()].cost;
                stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
            }
            else
            {
                while (eid != -1 and in_stack[adjTable[eid].adjvex])
                {
                    eid = adjTable[eid].next;
                }
                if (eid == -1)
                {
                    in_stack[stack.back()] = false;
                    stack.pop_back();
                    stack_e.pop_back();
                    dist -= adjTable[stack_e.back()].cost;
                    if (!stack_e.empty())
                        stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
                }
                else
                {
                    stack_e[stack_e.size()-1] = eid;
                    dist += adjTable[eid].cost;
                    int node = adjTable[eid].adjvex;
                    in_stack[node] = true;
                    stack.push_back(node);
                    stack_e.push_back(head[node].id);
                }
            }
        }
        else
        {
            in_stack[stack.back()] = false;
            stack.pop_back();
            stack_e.pop_back();
            dist -= adjTable[stack_e.back()].cost;
            stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
        }
    }
    return result;
}

vector<vector<int>*>* HisGraph::findAllTrailsByDFS(int a, int b, int MAX_DIST)
{
    vector<int> stack;
    vector<int> stack_e;
    int dist = 0;       // 记录栈中路径的距离
    stack.push_back(a);
    stack_e.push_back(head[a].id);
    vector<bool> in_stack(this->adjTable.size(), false);
    vector<vector<int>*>* result = new vector<vector<int>*>;
    while (!stack.empty())
    {
        int cur_node = stack.back();
        int eid = stack_e.back();
        if (dist <= MAX_DIST)
        {
            if (cur_node == b)
            {
                result->push_back(new vector<int>(stack));
                stack.pop_back();
                stack_e.pop_back();
                in_stack[stack_e.back()] = false;
                dist -= adjTable[stack_e.back()].cost;
                stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
            }
            else
            {
                while (eid != -1 and in_stack[eid])
                {
                    eid = adjTable[eid].next;
                }
                if (eid == -1)
                {
                    stack.pop_back();
                    stack_e.pop_back();
                    in_stack[stack_e.back()] = false;
                    dist -= adjTable[stack_e.back()].cost;
                    if (!stack_e.empty())
                        stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
                }
                else
                {
                    in_stack[eid] = true;
                    stack_e[stack_e.size()-1] = eid;
                    dist += adjTable[eid].cost;
                    int node = adjTable[eid].adjvex;
                    stack.push_back(node);
                    stack_e.push_back(head[node].id);
                }
            }
        }
        else
        {
            stack.pop_back();
            stack_e.pop_back();
            in_stack[stack_e.back()] = false;
            dist -= adjTable[stack_e.back()].cost;
            stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
        }
    }
    return result;
}

vector<vector<int>*>* HisGraph::findAllUTrailsByDFS(int a, int b, int MAX_DIST)
{
    vector<int> stack;
    vector<int> stack_e;
    int dist = 0;       // 记录栈中路径的距离
    stack.push_back(a);
    stack_e.push_back(head[a].id);
    vector<bool> in_stack((this->adjTable.size()>>1)+1, false);
    vector<vector<int>*>* result = new vector<vector<int>*>;
    while (!stack.empty())
    {
        int cur_node = stack.back();
        int eid = stack_e.back();
        if (dist <= MAX_DIST)
        {
            if (cur_node == b)
            {
                result->push_back(new vector<int>(stack));
                stack.pop_back();
                stack_e.pop_back();
                in_stack[stack_e.back()>>1] = false;
                dist -= adjTable[stack_e.back()].cost;
                stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
            }
            else
            {
                while (eid != -1 and in_stack[eid>>1])
                {
                    eid = adjTable[eid].next;
                }
                if (eid == -1)
                {
                    stack.pop_back();
                    stack_e.pop_back();
                    in_stack[stack_e.back()>>1] = false;
                    dist -= adjTable[stack_e.back()].cost;
                    if (!stack_e.empty())
                        stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
                }
                else
                {
                    in_stack[eid>>1] = true;
                    stack_e[stack_e.size()-1] = eid;
                    dist += adjTable[eid].cost;
                    int node = adjTable[eid].adjvex;
                    stack.push_back(node);
                    stack_e.push_back(head[node].id);
                }
            }
        }
        else
        {
            stack.pop_back();
            stack_e.pop_back();
            in_stack[stack_e.back()>>1] = false;
            dist -= adjTable[stack_e.back()].cost;
            stack_e[stack_e.size()-1] = adjTable[stack_e.back()].next;
        }
    }
    return result;
}

vector<HisGraph::EdgeSegment>* HisGraph::getRangeRoadSegments(int node, int r)
{
    vector<bool> vis(this->adjTable.size(), false);

    priority_queue<BFS_P> q;
    vector<HisGraph::EdgeSegment>* result = new vector<HisGraph::EdgeSegment>();

    int edge_id = head[node].id;
    while (edge_id != -1)
    {
        Edge* e = &adjTable[edge_id];
        q.push(BFS_P(e->adjvex, edge_id, e->cost));
        vis[edge_id] = true;
        edge_id = adjTable[edge_id].next;
    }

    while(!q.empty())
    {
        BFS_P cur_node = q.top();
        q.pop();
        if (cur_node.dist < r)
        {
            result->push_back(HisGraph::EdgeSegment(cur_node.e_id, 0, adjTable[cur_node.e_id].cost));
            edge_id = head[cur_node.v_id].id;
            while (edge_id != -1)
            {
                Edge* e = &adjTable[edge_id];
                if (!vis[edge_id])
                {
                    q.push(BFS_P(e->adjvex, edge_id, e->cost + cur_node.dist));
                    vis[edge_id] = true;
                }
                edge_id = adjTable[edge_id].next;
            }
        }
        else
        {
            result->push_back(HisGraph::EdgeSegment(cur_node.e_id, 0, adjTable[cur_node.e_id].cost - cur_node.dist + r));
        }
    }
    return result;
}

void HisGraph::save(const char* filename)
{
    ofstream f(filename);
    //boost::archive::text_oarchive oa(f);
    boost::archive::binary_oarchive oa(f);
    oa << *this;
    f.close();
}

void HisGraph::load(const char* filename)
{
    ifstream f(filename);
    //boost::archive::text_iarchive ia(f);
    boost::archive::binary_iarchive ia(f);
    ia >> *this;
    f.close();
}

HisGraph::HisGraph(const char* filename)
{
    load(filename);
}