#include "matching_result.h"

CRadians GetZAngleOrientation(CQuaternion orientation) {
   CRadians cZAngle, cYAngle, cXAngle;
   orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

void add_edge(int node1, int node2, Graph& g, vector<double>& cost, vector<pair<int, CVector2>>& positions){
    if(!g.AdjMat()[node1][node2]){
        g.AddEdge(node1, node2);
        CVector2 distance_vector1 = (positions[node1].second - positions[node2].second);
		cost[g.GetEdgeIndex(node1, node2)] = distance_vector1.Length();
    }
}

void AddHopToGraph(Graph& g, vector<double>& cost, vector<pair<int, CVector2>>& positions){
	int init_edges_num = g.GetNumEdges();
    for(unsigned i = 0; i < init_edges_num; i++){
        pair<int, int> first_edge = g.GetEdge(i);
		
        for(unsigned j = 0; j < init_edges_num; j++){
            if(i == j){
                continue;
            }
            pair<int, int> second_edge = g.GetEdge(j);
            if(first_edge.first == second_edge.first && first_edge.second < second_edge.second){
                add_edge(first_edge.second, second_edge.second, g, cost, positions);
            }
            else if(first_edge.first == second_edge.second && first_edge.second < second_edge.first){
                add_edge(first_edge.second, second_edge.first, g, cost, positions);
            }
            else if(first_edge.second == second_edge.second && first_edge.first < second_edge.first){
                add_edge(first_edge.first, second_edge.first, g, cost, positions);
            }
        }
    }	
}

MatchingResult GetMatchingResult(vector<CEPuck2Entity*> robots, double range) {
    std::vector<std::pair<int, CVector2>> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(i, position));
        orientations.push_back(std::make_pair(i, robots[i]->GetEmbodiedEntity().GetOriginAnchor().Orientation));
    }
    Graph G(number_of_robots);
    vector<double> cost(number_of_robots*number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        for(unsigned j=0;j<number_of_robots;j++){
            if(i != j) {
                CRadians cZAngle1 = GetZAngleOrientation(orientations[i].second);
                CRadians cZAngle2 = GetZAngleOrientation(orientations[j].second);
                CVector2 distance_vector1 = (positions[i].second - positions[j].second);

                Real angle_distance = 1 - (abs(cZAngle1.GetValue() - cZAngle2.GetValue()) / 3.141592);
                // if(distance_vector1.Length() < range){
                    G.AddEdge(i, j);
                    cost[G.GetEdgeIndex(i,j)] = distance_vector1.Length();
                // }
                // cost[G.GetEdgeIndex(i,j)] += + 0.00605 * angle_distance;
                // cost[G.GetEdgeIndex(i,j)] += + 0.01 * angle_distance;
            }
        }
    }
    AddHopToGraph(G, cost, positions);
    Matching M(G);
    pair< list<int>, double > solution = M.SolveMinimumCostPerfectMatching(cost);
    // cout << solution.second << endl;
    MatchingResult result = MatchingResult(G, M, cost, solution, robots);
    return result;
}

MatchingResult GetBestMatching(vector<CEPuck2Entity*> robots, double range){
    if(robots.size() % 2 == 0){
        return GetMatchingResult(robots, range);
    }
    vector<CEPuck2Entity*> robots_clone = robots;
    robots_clone.erase(robots_clone.begin());
    MatchingResult a = GetMatchingResult(robots_clone, range);
    MatchingResult* b = &a;
    unsigned j = 0;
    for(unsigned i = 1; i < robots.size(); i++) {
        robots_clone = robots;
        robots_clone.erase(std::next(robots_clone.begin(), i));
        MatchingResult result = GetMatchingResult(robots_clone, range);
        if(result._solution.second < b->_solution.second) {
            b = &result;
            j = i;
        }
    }
    robots_clone = robots;
    robots_clone.erase(robots_clone.begin()+j);
    MatchingResult bestMatchingResult = GetMatchingResult(robots_clone, range);
    return bestMatchingResult;
}