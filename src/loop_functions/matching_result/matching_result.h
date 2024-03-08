#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/loop_functions/minimum_cost_perfect_matching/Graph.h"
#include "src/loop_functions/minimum_cost_perfect_matching/Matching.h"
#include <list>
#include <iostream>
#include <fstream>

using namespace argos;
using namespace std;

struct MatchingResult{
    Graph _graph;
    vector<int> _matching;
    vector<CEPuck2Entity*> _robots;

    MatchingResult(Graph graph, vector<int> matching, vector<CEPuck2Entity*> robots):
        _graph(graph),
        _matching(matching),
        _robots(robots)
    {}
};

class Clusters{
    public:
        string ToString(){
            std::stringstream ss;
            ss << "[";
            for (size_t i = 0; i < _clusters.size(); ++i) {
                ss << "[";
                for (size_t j = 0; j < _clusters[i].size(); ++j) {
                    ss << _clusters[i][j];
                    if (j < _clusters[i].size() - 1) {
                        ss << ",";
                    }
                }
                ss << "]";
                if (i < _clusters.size() - 1) {
                    ss << ",";
                }
            }
            ss << "]";
            return ss.str(); 
        }
        void AddCluster(vector<int> cluster){
            _clusters.push_back(cluster);
        }
        int size(){
            return _clusters.size();
        }
    private:
        vector<vector<int>> _clusters;
};

CRadians GetZAngleOrientation(CQuaternion orientation);
MatchingResult GetMatchingResult(vector<CEPuck2Entity*> robots, double range);
// MatchingResult GetBestMatching(vector<CEPuck2Entity*> robots, double range);
list<int> SolveMinimumCostPerfectMatching(Graph& graph);