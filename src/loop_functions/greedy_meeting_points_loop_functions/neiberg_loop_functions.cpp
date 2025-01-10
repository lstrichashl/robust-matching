#include "neiberg_loop_functions.h"

CNeibergLoopFunctions::CNeibergLoopFunctions(){

}


void CNeibergLoopFunctions::Init(TConfigurationNode& t_tree) {
    CBasicLoopFunctions::Init(t_tree);
    // CGreedyMeetingPointsEpuck::Reset();
}

void CNeibergLoopFunctions::Reset(){
    CBasicLoopFunctions::Reset();
}

// void CNeibergLoopFunctions::PreStep(){

// }

void PrintGraph(const AugmentationGraph& graph) {
    std::cout << "AugmentationGraph:\n";;
    for (const auto& node : graph.paths) {
        std::cout << "Path (Gain: " << node.gain << "): ";
        for (UInt8 v : node.nodes) {
            std::cout << static_cast<int>(v) << " ";
        }
        std::cout << "\n";

        if (graph.edges.find(node) != graph.edges.end()) {
            std::cout << "Neighbors: ";
            for (const auto& neighbor : graph.edges.at(node)) {
                std::cout << "(Gain: " << neighbor.gain << ") ";
                for (UInt8 v : neighbor.nodes) {
                    std::cout << static_cast<int>(v) << " ";
                }
                std::cout << "\n";
            }
        }
        std::cout << "------------------\n";;
    }
}


void PrintAccumulatedMIS(const std::unordered_set<AugmentationPath, AugmentationPathHash>& accumulatedMIS) {
    std::cout << "Accumulated MIS:\n";
    for (const auto& node : accumulatedMIS) {
        std::cout << "Path (Gain: " << node.gain << "): ";
        for (UInt8 v : node.nodes) {
            std::cout << static_cast<int>(v) << " ";
        }
        std::cout << "\n";
    }
}

AugmentationGraph CombineAugmentationGraphs(const std::vector<AugmentationGraph>& robotGraphs) {
    AugmentationGraph combinedGraph;

    // Use a set to store unique paths
    std::unordered_set<AugmentationPath, AugmentationPathHash> uniquePaths;

    for (const auto& graph : robotGraphs) {
        for (const auto& path : graph.paths) {
            // Normalize the path nodes by sorting them
            AugmentationPath normalizedPath = path;
            // std::sort(normalizedPath.nodes.begin(), normalizedPath.nodes.end());

            // Add the normalized path to the unique set
            auto inserted = uniquePaths.insert(normalizedPath);
            if (inserted.second) {
                // If the path was newly inserted, set its gain
                combinedGraph.paths.insert(normalizedPath);
            } else {
                // If the path already exists, accumulate the gain
                auto& existingPath = const_cast<AugmentationPath&>(*inserted.first);
                existingPath.gain += normalizedPath.gain;
            }
        }
    }

    for (const auto& path1 : combinedGraph.paths) {
        for (const auto& path2 : combinedGraph.paths) {
            std::unordered_set<UInt8> nodes1(path1.nodes.begin(), path1.nodes.end());
            for (UInt8 node : path2.nodes) {
                if (nodes1.count(node)) {
                    combinedGraph.edges[path1].insert(path2);
                    combinedGraph.edges[path2].insert(path1);
                    break;
                }
            }
        }
    }

    return combinedGraph;
    
}

AugmentationGraph CombineAllGraphs(const std::unordered_map<UInt8, NeighborhoodGraphController*>&  robotControllers) {
    std::vector<AugmentationGraph> robotGraphs;

    // Gather graphs from all robots
    for (const auto& pair : robotControllers) {
        robotGraphs.push_back(pair.second->GetAugmentationGraph());
    }

    // Combine the graphs
    AugmentationGraph combinedGraph = CombineAugmentationGraphs(robotGraphs);
    return combinedGraph;
}


std::unordered_set<AugmentationPath, AugmentationPathHash> CalculateMaximalIndependentSet(const AugmentationGraph& graph) {
    std::unordered_set<AugmentationPath, AugmentationPathHash> mis; // Stores the maximal independent set
    std::unordered_set<AugmentationPath, AugmentationPathHash> remaining_paths = graph.paths; // Paths not yet processed
    while (!remaining_paths.empty()) {
        const AugmentationPath& selected_path = *remaining_paths.begin();
        mis.insert(selected_path);
        if (graph.edges.find(selected_path) != graph.edges.end()) {
            for (const auto& neighbor : graph.edges.at(selected_path)) {
                remaining_paths.erase(neighbor);
            }
        }
        remaining_paths.erase(selected_path);
    }

    return mis;
}

std::unordered_set<AugmentationPath, AugmentationPathHash> CalculateSubsetW(const AugmentationGraph& graph) {
    std::unordered_set<AugmentationPath, AugmentationPathHash> subsetW;

    for (const auto& node : graph.paths) {
        bool satisfiesCondition = true;
        for (const auto& neighbor : graph.edges.at(node)) {
            if (neighbor.gain > 2 * node.gain) {
                satisfiesCondition = false;
                break;
            }
        }

        if (satisfiesCondition) {
            subsetW.insert(node);
        }
    }

    return subsetW;
}

AugmentationGraph CalculateInducedSubgraph(const AugmentationGraph& graph, 
                                           const std::unordered_set<AugmentationPath, AugmentationPathHash>& subsetW) {
    AugmentationGraph inducedGraph;

    for (const auto& node : subsetW) {
        inducedGraph.paths.insert(node);

        if (graph.edges.find(node) != graph.edges.end()) {
            for (const auto& neighbor : graph.edges.at(node)) {
                if (subsetW.find(neighbor) != subsetW.end()) {
                    inducedGraph.edges[node].insert(neighbor);
                }
            }
        }
    }

    return inducedGraph;
}

void RemoveAdjacentNodes(AugmentationGraph& graph, 
                         const std::unordered_set<AugmentationPath, AugmentationPathHash>& mis) {
    std::unordered_set<AugmentationPath, AugmentationPathHash> nodesToRemove;

    for (const auto& node : mis) {
        nodesToRemove.insert(node); // Add the MIS nodes themselves
        if (graph.edges.find(node) != graph.edges.end()) {
            for (const auto& neighbor : graph.edges.at(node)) {
                nodesToRemove.insert(neighbor); // Add all neighbors of MIS nodes
            }
        }
    }

    // Remove the nodes and their edges
    for (const auto& node : nodesToRemove) {
        graph.paths.erase(node);
        graph.edges.erase(node);
    }

    // Remove edges pointing to the removed nodes
    for (auto& [node, neighbors] : graph.edges) {
        for (const auto& toRemove : nodesToRemove) {
            neighbors.erase(toRemove);
        }
    }
}


std::unordered_set<AugmentationPath, AugmentationPathHash> ProcessAugmentationGraph(AugmentationGraph& graph) {
    std::unordered_set<AugmentationPath, AugmentationPathHash> accumulatedMIS; // Store cumulative MIS nodes

    for(int t = 0; t < 8; t++) { // TODO: t < log_2(n*l^2)
        // Step 1: Calculate subset W
        auto subsetW = CalculateSubsetW(graph);
        // cout << "subsetW" << endl;
        // for (const auto& node : subsetW) {
        //     std::cout << "Path (Gain: " << node.gain << "): ";
        // for (UInt8 v : node.nodes) {
        //     std::cout << static_cast<int>(v) << " ";
        // }
        // std::cout << "\n";
        // }
        // cout << "--------subsetW------" << endl;

        // Step 2: Calculate the induced subgraph G'
        auto inducedGraph = CalculateInducedSubgraph(graph, subsetW);
        // PrintGraph(inducedGraph);

        // // Step 3: Calculate the maximal independent set (MIS) in G'
        auto mis = CalculateMaximalIndependentSet(inducedGraph);
        

        // // Add all nodes in the MIS to the cumulative set A
        accumulatedMIS.insert(mis.begin(), mis.end());
        // // Step 4: Remove all nodes from G that are adjacent to nodes in the MIS
        RemoveAdjacentNodes(graph, mis);
    }

    return accumulatedMIS;
}

void AugmentMatching(
    const std::vector<UInt8>& path, 
    std::unordered_map<UInt8, NeighborhoodGraphController*>& robotControllers) {


    // Track whether the current edge is in the matching or not
    bool inMatching = false;

    // for (size_t i = 0; i < path.size() - 1; i++) {
    //     UInt8 u = path[i];
    //     auto robotU = robotControllers[u];
    //     if(robotU->m_matchedRobotId != UINT8_MAX){
    //         robotControllers[robotU->m_matchedRobotId]->m_matchedRobotId = UINT8_MAX;
    //     }
    //     robotU->m_matchedRobotId = UINT8_MAX;
    // }
    // Traverse the path to flip the matching edges
    for (size_t i = 0; i < path.size() - 1; i+=2) {
        UInt8 u = path[i];
        UInt8 v = path[i + 1];
        auto robotU = robotControllers[u];
        auto robotV = robotControllers[v];
        if(i % 2 == 0){
            if(robotU->m_matchedRobotId != UINT8_MAX){
                robotControllers[robotU->m_matchedRobotId]->m_matchedRobotId = UINT8_MAX;
            }
            if(robotV->m_matchedRobotId != UINT8_MAX){
                robotControllers[robotV->m_matchedRobotId]->m_matchedRobotId = UINT8_MAX;
            }
            robotU->m_matchedRobotId = v;
            robotV->m_matchedRobotId = u;
        }
        else{
            robotU->m_matchedRobotId = UINT8_MAX;
            robotV->m_matchedRobotId = UINT8_MAX;
        }
    }
}



void CNeibergLoopFunctions::PostStep(){
    std::unordered_map<UInt8, NeighborhoodGraphController*> robotControllers;
    for(unsigned i = 0; i < m_robots.size(); i++){
        NeighborhoodGraphController& cController1 = dynamic_cast<NeighborhoodGraphController&>(GetControllableEntity3(m_robots[i])->GetController());
        robotControllers[cController1.m_uRobotId] = &cController1;
    }
    auto it = robotControllers.begin();
    if(it->second->m_uCurrentHop == it->second->m_uMaxDistance+1){
        AugmentationGraph graph = CombineAllGraphs(robotControllers);
        std::unordered_set<AugmentationPath, AugmentationPathHash> accumulatedMIS = ProcessAugmentationGraph(graph);
        PrintAccumulatedMIS(accumulatedMIS);
        for (const auto& path : accumulatedMIS) {
            AugmentMatching(path.nodes, robotControllers);
        }
        // for(const auto& [id, controller] : robotControllers){
        //     controller->StartMatching();
        // }
        // std::cout << "Robot Match Status:\n";
        // for (const auto& [robotId, controller] : robotControllers) {
        // std::cout << "Robot " << static_cast<int>(robotId) 
        //           << " -> " 
        //           << (controller->m_matchedRobotId == UINT8_MAX ? "Unmatched" : 
        //               std::to_string(static_cast<int>(controller->m_matchedRobotId))) 
        //           << "\n";
        // }
    }
}




REGISTER_LOOP_FUNCTIONS(CNeibergLoopFunctions, "neiberg_loop_functions")
