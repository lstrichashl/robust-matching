#include "Matching.h"
#include <fstream>
#include "Graph.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
using namespace std;

pair< Graph, vector<double> > CreateRandomGraph()
{
	//random seed
	int x;
	cin >> x;
	srand( x );

	//Please see Graph.h for a description of the interface
	int n = 50;

	Graph G(n);
	vector<double> cost;
	for(int i = 0; i < n; i++)
		for(int j = i+1; j < n; j++)
			if(rand()%10 == 0)
			{
				G.AddEdge(i, j);
				cost.push_back(rand()%1000);
			}

	return make_pair(G, cost);
}

Graph ReadGraph(string filename)
{
	//Please see Graph.h for a description of the interface

	ifstream file;
	file.open(filename.c_str());

	string s;
	getline(file, s);
	stringstream ss(s);
	int n;
	ss >> n;
	getline(file, s);
	ss.str(s);
	ss.clear();
	int m;
	ss >> m;

	Graph G(n);
	for(int i = 0; i < m; i++)
	{
		getline(file, s);
		ss.str(s);
		ss.clear();
		int u, v;
		ss >> u >> v;

		G.AddEdge(u, v);
	}

	file.close();
	return G;
}

pair< Graph, vector<double> > ReadWeightedGraph(string filename)
{
	//Please see Graph.h for a description of the interface

	ifstream file;
	file.open(filename.c_str());

	string s;
	getline(file, s);
	stringstream ss(s);
	int n;
	ss >> n;
	getline(file, s);
	ss.str(s);
	ss.clear();
	int m;
	ss >> m;

	Graph G(n);
	vector<double> cost(n*n);
	for(int i = 0; i < m; i++)
	{
		getline(file, s);
		ss.str(s);
		ss.clear();
		int u, v;
		double c;
		ss >> u >> v >> c;
		G.AddEdge(u, v);
		cost[G.GetEdgeIndex(u, v)] = c;
	}

	file.close();
	return make_pair(G, cost);
}


void add_edge(int node1, int node2, Graph& g){
	if(!g.AdjMat()[node1][node2]){
		g.AddEdge(node1, node2);
		g.costs[g.GetEdgeIndex(node1, node2)] = 5;
	}
}

void AddHopToGraph(Graph& g){
	int init_edges_num = g.GetNumEdges();
    for(unsigned i = 0; i < init_edges_num; i++){
        pair<int, int> first_edge = g.GetEdge(i);
        for(unsigned j = 0; j < init_edges_num; j++){
            if(i == j){
                continue;
            }
            pair<int, int> second_edge = g.GetEdge(j);
            if(first_edge.first == second_edge.first && first_edge.second < second_edge.second){
                add_edge(first_edge.second, second_edge.second, g);
            }
            else if(first_edge.first == second_edge.second && first_edge.second < second_edge.first){
                add_edge(first_edge.second, second_edge.first, g);
            }
            else if(first_edge.second == second_edge.second && first_edge.first < second_edge.first){
                add_edge(first_edge.first, second_edge.first, g);
            }
        }
    }	
}

Graph remove_node(Graph& graph, int v){
	vector<pair<int,int> > edges;
	Graph new_graph(graph.GetNumVertices()-1);
	for(int j = 0; j < graph.edges.size(); j++){
		pair<int,int> edge = graph.edges[j];
		if(edge.first != v && edge.second != v && edge.first < edge.second){
			int first_node = edge.first, second_node = edge.second;
			if(first_node > v){
				first_node--;
			}
			if(second_node > v){
				second_node--;
			}
			new_graph.AddEdge(first_node, second_node, graph.GetCost(edge.first, edge.second));
		}
	}
	return new_graph;
}

void print_graph(Graph g){
	for (int v = 0; v < g.GetNumVertices(); ++v) {
		cout << "Vertex " << v << ": ";
		for (int neighbor : g.AdjList(v))
			cout << neighbor << " ";
		cout << endl;
	}
}

pair< list<int>, double > MinimumMatchingSubgraph(Graph graph){
	double minimum_cost = 1000000000;
	list<int> minimum_matching;
	int minimum_index = -1;
	Graph min_subgraph;
    for(int i = 0; i < graph.GetNumVertices(); i++){
		Graph subgraph = remove_node(graph, i);
		Matching M(subgraph);
		try{
			pair< list<int>, double > solution = M.SolveMinimumCostPerfectMatching(subgraph.costs);

			list<int> matching = solution.first;
			double obj = solution.second;

			if(minimum_cost > solution.second){
				minimum_matching = solution.first;
				minimum_cost = solution.second;
				minimum_index = i;
				min_subgraph = subgraph;
			}

		}catch(const char * msg){
			if(strcmp(msg,"Error: The graph does not have a perfect matching") != 0){
				throw msg;
			}
			cout << "error" << endl;
		}
	}
	if(minimum_cost == 1000000000){
		throw "Error: The graph does not have a perfect matching 2";
	}
	list<int> minimum_matching2;
	for(list<int>::iterator it = minimum_matching.begin(); it != minimum_matching.end(); it++){
		pair<int, int> edge = min_subgraph.GetEdge( *it );
		int first_node = edge.first, second_node = edge.second;
		if(first_node >= minimum_index){
			first_node++;
		}
		if(second_node >= minimum_index){
			second_node++;
		}
		minimum_matching2.push_back(graph.GetEdgeIndex(first_node, second_node));
	}
	pair< list<int>, double > solution = pair< list<int>, double >(minimum_matching2, minimum_cost);
	return solution;
}

list<int> MatchingForComponents(Graph& graph, vector<Graph>& components, vector<vector<int> >& comp_vertecies){
	list<int> matching;
	for(int i = 0; i < components.size(); i++){
		pair< list<int>, double > solution;
		if(components[i].GetNumVertices() % 2 == 0){
			Matching M(components[i]);
			solution = M.SolveMinimumCostPerfectMatching(components[i].costs);
		}else{
			solution = MinimumMatchingSubgraph(components[i]);
		}
		cout << solution.second << endl;
		for(list<int>::iterator it = solution.first.begin(); it != solution.first.end(); it++){
			pair<int, int> e = components[i].GetEdge( *it );
			matching.push_back(graph.GetEdgeIndex(comp_vertecies[i][e.first], comp_vertecies[i][e.second]));
		}
	}
	return matching;
}

list<int> GetMatching(Graph& graph){
	AddHopToGraph(graph);
	map<int, int> vertexMap;
	vector<vector<int> > comp_vertecies = graph.findConnectedComponents(vertexMap);
	vector<Graph> components;

	for(int i = 0; i < comp_vertecies.size(); i++) {
		Graph component(comp_vertecies[i].size());
		vector<double> comp_cost;
		for (int v = 0; v < comp_vertecies[i].size(); v++) {
            for (int neighbor : graph.AdjList(comp_vertecies[i][v])){
				component.AddEdge(vertexMap[comp_vertecies[i][v]], vertexMap[neighbor], graph.costs[graph.GetEdgeIndex(comp_vertecies[i][v], neighbor)]);
			}
        }
		components.push_back(component);
	}
	list<int> matching = MatchingForComponents(graph, components, comp_vertecies);
	return matching;
}

void MinimumCostPerfectMatchingExample(string filename)
{
	Graph G;
	vector<double> cost;
	
	pair< Graph, vector<double> > p = ReadWeightedGraph(filename);
	G = p.first;
	G.costs = p.second;
	

	list<int> matching = GetMatching(G);
	// cout << "Optimal matching cost: " << obj << endl;
	cout << "Edges in the matching:" << endl;
	for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	{
		pair<int, int> e = G.GetEdge( *it );
		double c = G.GetCost(e.first, e.second);

		cout << e.first << " " << e.second << " " << c << endl;
	}
}

void MaximumMatchingExample(string filename)
{
	Graph G = ReadGraph(filename);
	Matching M(G);

	list<int> matching;
	matching = M.SolveMaximumMatching();

	cout << "Number of edges in the maximum matching: " << matching.size() << endl;
	cout << "Edges in the matching:" << endl;
	for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	{
		pair<int, int> e = G.GetEdge( *it );

		cout << e.first << " " << e.second << endl;
	}
}

int main(int argc, char* argv[])
{
	string filename = "";
	string algorithm = "";

	int i = 1;
	while(i < argc)
	{
		string a(argv[i]);
		if(a == "-f")
			filename = argv[++i];
		else if(a == "--minweight")
			algorithm = "minweight";
		else if(a == "--max")
			algorithm = "max";
		i++;
	}

	// if(filename == "" || algorithm == "")
	// {
	// 	cout << "usage: ./example -f <filename> <--minweight | --max>" << endl;
	// 	cout << "--minweight for minimum weight perfect matching" << endl;
	// 	cout << "--max for maximum cardinality matching" << endl;
	// 	cout << "file format:" << endl;
	// 	cout << "the first two lines give n (number of vertices) and m (number of edges)," << endl;
	// 	cout << "followed by m lines, each with a tuple (u, v [, c]) representing the edges," << endl;
	//    	cout << "where u and v are the endpoints (0-based indexing) of the edge and c is its cost" << endl;	
	// 	cout << "the cost is optional if --max is specified" << endl;
	// 	return 1;
	// }

	try
	{
		// if(algorithm == "minweight")
			MinimumCostPerfectMatchingExample(filename);
		// else
			// MaximumMatchingExample(filename);
	}
	catch(const char * msg)
	{
		cout << msg << endl;
		return 1;
	}

	return 0;
}



