#include "matching_result.h"

CRadians GetZAngleOrientation(CQuaternion orientation) {
   CRadians cZAngle, cYAngle, cXAngle;
   orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

void add_edge(int node1, int node2, Graph& g, vector<CVector2>& positions){
    if(!g.AdjMat()[node1][node2]){
        CVector2 distance_vector1 = (positions[node1] - positions[node2]);
        g.AddEdge(node1, node2, distance_vector1.Length());
    }
}

void AddHopToGraph(Graph& g, vector<CVector2>& positions){
	int init_edges_num = g.GetNumEdges();
    for(unsigned i = 0; i < init_edges_num; i++){
        pair<int, int> first_edge = g.GetEdge(i);
		
        for(unsigned j = 0; j < init_edges_num; j++){
            if(i == j){
                continue;
            }
            pair<int, int> second_edge = g.GetEdge(j);
            if(first_edge.first == second_edge.first && first_edge.second < second_edge.second){
                add_edge(first_edge.second, second_edge.second, g, positions);
            }
            else if(first_edge.first == second_edge.second && first_edge.second < second_edge.first){
                add_edge(first_edge.second, second_edge.first, g, positions);
            }
            else if(first_edge.second == second_edge.second && first_edge.first < second_edge.first){
                add_edge(first_edge.first, second_edge.first, g, positions);
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
		// cout << solution.second << endl;
		for(list<int>::iterator it = solution.first.begin(); it != solution.first.end(); it++){
			pair<int, int> e = components[i].GetEdge( *it );
			matching.push_back(graph.GetEdgeIndex(comp_vertecies[i][e.first], comp_vertecies[i][e.second]));
		}
	}
	return matching;
}

list<int> SolveMinimumCostPerfectMatching(Graph& graph){
	unordered_map<int, int> vertexMap;
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


MatchingResult GetMatchingResult(vector<CEPuck2Entity*> robots, double range) {
    std::vector<CVector2> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(position);
        orientations.push_back(std::make_pair(i, robots[i]->GetEmbodiedEntity().GetOriginAnchor().Orientation));
    }
    Graph G(number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        for(unsigned j=0;j<number_of_robots;j++){
            if(i != j) {
                CRadians cZAngle1 = GetZAngleOrientation(orientations[i].second);
                CRadians cZAngle2 = GetZAngleOrientation(orientations[j].second);
                CVector2 distance_vector1 = (positions[i] - positions[j]);

                Real angle_distance = 1 - (abs(cZAngle1.GetValue() - cZAngle2.GetValue()) / 3.141592);
                if(distance_vector1.Length() < range){
                    G.AddEdge(i, j, distance_vector1.Length());
                }
                // cost[G.GetEdgeIndex(i,j)] += + 0.00605 * angle_distance;
                // cost[G.GetEdgeIndex(i,j)] += + 0.01 * angle_distance;
            }
        }
    }
    AddHopToGraph(G, positions);
    list<int> matching = SolveMinimumCostPerfectMatching(G);
    vector<int> v_matching;
    vector<CEPuck2Entity*> robots_in_matching;
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++){
		pair<int, int> e = G.GetEdge( *it );
		double c = G.GetCost(e.first, e.second);

		// cout << e.first << " " << e.second << " " << c << endl;
        v_matching.push_back(*it);
        robots_in_matching.push_back(robots[e.first]);
        robots_in_matching.push_back(robots[e.second]);
	}
    MatchingResult result = MatchingResult(G, v_matching, robots_in_matching);
    return result;
}

// MatchingResult GetBestMatching(vector<CEPuck2Entity*> robots, double range){
//     if(robots.size() % 2 == 0){
//         return GetMatchingResult(robots, range);
//     }
//     vector<CEPuck2Entity*> robots_clone = robots;
//     robots_clone.erase(robots_clone.begin());
//     MatchingResult a = GetMatchingResult(robots_clone, range);
//     MatchingResult* b = &a;
//     unsigned j = 0;
//     for(unsigned i = 1; i < robots.size(); i++) {
//         robots_clone = robots;
//         robots_clone.erase(std::next(robots_clone.begin(), i));
//         MatchingResult result = GetMatchingResult(robots_clone, range);
//         if(result._solution.second < b->_solution.second) {
//             b = &result;
//             j = i;
//         }
//     }
//     robots_clone = robots;
//     robots_clone.erase(robots_clone.begin()+j);
//     MatchingResult bestMatchingResult = GetMatchingResult(robots_clone, range);
//     return bestMatchingResult;
// }





























