#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/robust_matching.h"
#include <list>

void CMatchingLoopFunctions::Init(TConfigurationNode& t_tree) {
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
}

void CMatchingLoopFunctions::PreStep(){
    std::vector<std::pair<int, CVector2>> positions;
    int number_of_robots = m_robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(m_robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                m_robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(i, position));
    }
    std::vector<std::pair<int, CVector2>>::iterator it1;
    std::vector<std::pair<int, CVector2>>::iterator it2;
    Graph G(number_of_robots);
	vector<double> cost(number_of_robots*number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        for(unsigned j=0;j<number_of_robots;j++){
            Real distance = (positions[i].second - positions[j].second).Length();
            G.AddEdge(i, j);
            cost[G.GetEdgeIndex(i,j)] = distance;
        }
    }
    Matching M(G);
    pair< list<int>, double > solution = M.SolveMinimumCostPerfectMatching(cost);

	list<int> matching = solution.first;
	double matching_cost = solution.second;

	cout << "Optimal matching cost: " << matching_cost << endl;
	for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	{
		pair<int, int> edge = G.GetEdge( *it );
        CEPuck2Entity* robot1 = m_robots[edge.first];
        CEPuck2Entity* robot2 = m_robots[edge.second];
        CRobustMatching& cController1 = dynamic_cast<CRobustMatching&>(robot1->GetControllableEntity().GetController());
        CRobustMatching& cController2 = dynamic_cast<CRobustMatching&>(robot2->GetControllableEntity().GetController());
        cController1.mate = robot2;
        cController2.mate = robot1;
	}

    m_solution = solution;
    m_robotGraph = G;
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
