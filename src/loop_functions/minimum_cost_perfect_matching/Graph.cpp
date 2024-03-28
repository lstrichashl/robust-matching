#include "Graph.h"

Graph::Graph(int n, const vector< pair<int, int> > & edges):
	n(n),
	m(edges.size()),
	adjMat(n, vector<bool>(n, false)),
	adjList(n),
	edges(),
	edgeIndex(n, vector<int>(n, -1)),
	costs(n*n)
{
	for(int i = 0; i < edges.size(); i++)
	{
		int u = edges[i].first;
		int v = edges[i].second;

		AddEdge(u, v);
	}
}

Graph::Graph(vector<argos::CVector2> positions, double range):
	Graph(positions.size())
{
	for(unsigned i=0; i<positions.size(); i++){
		for(unsigned j=0;j<positions.size();j++){
			if(i != j) {
				argos::CVector2 distance_vector1 = (positions[i] - positions[j]);
				if(distance_vector1.Length() < range){
					AddEdge(i, j, distance_vector1.Length());
				}
			}
		}
	}
}

pair<int, int> Graph::GetEdge(int e) const
{
	if(e > (int)edges.size())
		throw "Error: edge does not exist";

	return edges[e];
}

double Graph::GetCost(int u, int v){
	return costs[GetEdgeIndex(u,v)];
}

int Graph::GetEdgeIndex(int u, int v) const
{
	if( u > n or
		v > n )
		throw "Error: vertex does not exist";

	if(edgeIndex[u][v] == -1)
		throw "Error: edge does not exist";

	return edgeIndex[u][v];
}

void Graph::AddVertex()
{
	for(int i = 0; i < n; i++)
	{
		adjMat[i].push_back(false);
		edgeIndex[i].push_back(-1);
	}
	n++;
	adjMat.push_back( vector<bool>(n, false) );
	edgeIndex.push_back( vector<int>(n, -1) );
	adjList.push_back( list<int>() );
}

void Graph::AddEdge(int u, int v, double cost)
{
	if( u > n or
		v > n )
		throw "Error: vertex does not exist";

	if(adjMat[u][v]) return;

	adjMat[u][v] = adjMat[v][u] = true;
	adjList[u].push_back(v);
	adjList[v].push_back(u);

	edges.push_back(pair<int, int>(u, v));
	edgeIndex[u][v] = edgeIndex[v][u] = m++;
	costs[edgeIndex[u][v]] = cost;
}

const list<int> & Graph::AdjList(int v) const
{
	if(v > n)
		throw "Error: vertex does not exist";

	return adjList[v];
}

const vector< vector<bool> > & Graph::AdjMat() const
{
	return adjMat;
}


    // Depth First Search (DFS) algorithm
void Graph::DFS(int v, vector<bool>& visited, vector<int>& component) {
	visited[v] = true;
	component.push_back(v);

	// Visit all adjacent vertices of v
	for (int neighbor : adjList[v]) {
		if (!visited[neighbor])
			DFS(neighbor, visited, component);
	}
}


 // Function to find connected components
vector<vector<int> > Graph::findConnectedComponents(unordered_map<int, int>& vertexMap) {
	vector<bool> visited(n, false);
	vector<vector<int> > comps;
	// unordered_map<int, int> vertexMap; // Map to store old vertex IDs to new vertex IDs for each component

	for (int v = 0; v < n; ++v) {
		if (!visited[v]) {
			vector<int> component;
			DFS(v, visited, component);
			comps.push_back(component);
			// Create a mapping from old vertex IDs to new vertex IDs for this component
			for (size_t i = 0; i < component.size(); ++i) {
				vertexMap[component[i]] = i;
			}
		}
	}
	return comps;
}