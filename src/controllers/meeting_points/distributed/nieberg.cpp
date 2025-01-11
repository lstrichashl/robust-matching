#include "nieberg.h"
#include <queue>

NeighborhoodGraphController::NeighborhoodGraphController(){
    
}

void NeighborhoodGraphController::Init(TConfigurationNode& t_node) {
    BaseConrtoller::Init(t_node);
    // Initialize actuators and sensors
    // m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
    // m_pcRABSens = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );

    // Initialize robot parameters
    GetNodeAttribute(t_node, "max_distance", m_uMaxDistance);
    GetNodeAttribute(t_node, "range", m_visRange);
    m_visRange *= 100;
    time = 0;

    string id = GetId();
    m_uRobotId = stoi(id);
    m_uCurrentHop = 0;
    m_matchedRobotId = UINT8_MAX;
    m_phase = Construct_Augmentation_Graph;
    // if(m_uRobotId == 10){
    //     m_matchedRobotId = 12;
    // }    
    // if(m_uRobotId == 12){
    //     m_matchedRobotId = 10;
    // }    
    // if(m_uRobotId == 17){
    //     m_matchedRobotId = 18;
    // }    
    // if(m_uRobotId == 18){
    //     m_matchedRobotId = 17;
    // }

    // Add self to graph
    m_sNodes.insert(m_uRobotId);
}

AugmentationGraph NeighborhoodGraphController::BuildAugmentationGraph(const std::vector<std::vector<UInt8>>& paths,
                                         const std::unordered_map<size_t, Real>& gains) {
    AugmentationGraph graph;

    // Step 1: Convert paths and gains into AugmentationPath structs and insert into the graph
    for (size_t i = 0; i < paths.size(); ++i) {
        AugmentationPath path;
        path.nodes = paths[i];
        path.gain = gains.at(i); // Assuming gains are indexed by the same indices as paths
        path.priority = pcRNG->Uniform(CRange<Real>(0, 1));
        // auto minIt = std::min_element(path.nodes.begin(), path.nodes.end());
        // int minNodeId = static_cast<int>(*minIt);

        // if(minNodeId == m_uRobotId){
        //     graph.my_paths.insert(path);
        // }
        // else{
        //     graph.others_paths.insert(path);
        // }
        graph.paths.insert(path);
    }

    for (const auto& path1 : graph.paths) {
        for (const auto& path2 : graph.paths) {
            std::unordered_set<UInt8> nodes1(path1.nodes.begin(), path1.nodes.end());
            for (UInt8 node : path2.nodes) {
                if (nodes1.count(node)) {
                    graph.edges[path1].insert(path2);
                    graph.edges[path2].insert(path1);
                    break;
                }
            }
        }
    }

    return graph;
}


void NeighborhoodGraphController::ControlStep(){
    time += 1;
    // if(m_uRobotId == 15 && m_phase != Move){
        // cout << "Nodes: ";
        // for (UInt8 node : m_sNodes) {
        //     cout << node << " ";
        // }
    //     cout << "\nEdges: ";
    //     for (const auto& edge : m_sEdges) {
    //         cout << "(" << edge.node1 << ", " << edge.node2 << "," <<edge.isMatching << ") ";
    //     }
    // }
    if(m_phase == Construct_Augmentation_Graph){
        if (m_uCurrentHop < m_uMaxDistance) {
            ExchangeNeighborhood();
            m_uCurrentHop++;
        } else if(m_uCurrentHop == m_uMaxDistance) {
            std::vector<std::vector<UInt8>> paths;
            std::unordered_map<size_t, Real> gains;

            // AddHopToGraph();
            FindAugmentationPaths(m_uRobotId, m_uMaxDistance, paths, gains);
            m_graph = BuildAugmentationGraph(paths, gains);
            m_vAugmentationPaths = paths;
            m_vPathGains = gains;

            m_uCurrentHop++;
        }
        else{
            m_phase = MAIN_LOOP;
            m_uCurrentHop = 0;
            accumulatedMIS.clear();
        }
    }
    else if(m_phase == MAIN_LOOP){
        if(m_uCurrentHop < m_uMaxDistance){
            PropegratePaths(m_vAugmentationPaths);
            m_uCurrentHop++;
        } else if(m_uCurrentHop == m_uMaxDistance) {
            m_phase = Maximal_Independent_Set;
            std::unordered_set<AugmentationPath, AugmentationPathHash> mis = CalculateMIS();
            accumulatedMIS.insert(mis.begin(), mis.end());
            RemoveAdjacentNodes(m_graph, mis);
            m_uCurrentHop++;
        }
    }
    else if(m_phase == Augmenting){
        vector<vector<UInt8>> paths;
        if(m_uCurrentHop == 0){
            for(auto& path : accumulatedMIS){
                paths.push_back(path.nodes);
            }
        }
        if(m_uCurrentHop < m_uMaxDistance){
            PropegratePaths(paths);
            m_uCurrentHop++;
        }
    }
    if(m_phase == Move && m_matchedRobotId != UINT8_MAX && m_eState == STATE_ALONE) {
        BaseConrtoller::Communicate();
        CVector2 to_mate = m_meeting_point - m_position;
        CRadians cZAngle, cYAngle, cXAngle;
        m_orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
        m_heading = to_mate.Rotate(-cZAngle);
        Real distance_to_meeting_point = (m_meeting_point - m_position).Length();
        if(distance_to_meeting_point-PAIRING_THRESHOLD/2 < 0) {
            m_heading = CVector2::ZERO;
        }
        else{
            m_heading = m_heading.Normalize();
            m_heading += FlockingVector();
        }
    }
    else{
        m_heading = CVector2::ZERO;
    }
    if(m_phase == Move && time % m_iterrationtime == 0){
        if(m_eState == STATE_ALONE){
            StartMatching();
            m_matched_robot_indexes.insert(m_matchedRobotId);
            m_matchedRobotId = UINT8_MAX;
        }else{
            m_phase = Idle;
            m_sNodes.clear();
            m_sEdges.clear();
            m_sNodes.insert(m_uRobotId);
            m_uCurrentHop = 0;
            std::unordered_set<AugmentationPath, AugmentationPathHash> paths; // Unique paths
            std::unordered_map<AugmentationPath, std::unordered_set<AugmentationPath, AugmentationPathHash>, AugmentationPathHash> edges; // Adjacency list

            m_graph.paths = paths;
            m_graph.edges = edges;
            m_pcRABAct->Reset();
            m_pcRABSens->Reset();
        }
    }


    handleFaultBehaviour();
    SetWheelSpeedsFromVector(m_heading);
    // else if(m_uCurrentHop  == m_uMaxDistance * 2) {
    //     cout << "matching:"<< m_uRobotId << " " << m_matchedRobotId<<endl;
    //     StartMatching();
    // }
    // else{
    //     // cout << "idle" << endl;
    //     m_uCurrentHop++;
    // }

}

void NeighborhoodGraphController::RemoveAdjacentNodes(AugmentationGraph& graph, 
                            const std::unordered_set<AugmentationPath, AugmentationPathHash>& mis){

}

unordered_set<AugmentationPath, AugmentationPathHash> NeighborhoodGraphController::CalculateMIS(){
    unordered_set<AugmentationPath, AugmentationPathHash> mis;
    return mis;
}

void NeighborhoodGraphController::StartMatching(){
    m_sNodes.clear();
    m_sEdges.clear();
    m_sNodes.insert(m_uRobotId);
    m_uCurrentHop = 0;
    std::unordered_set<AugmentationPath, AugmentationPathHash> paths; // Unique paths
    std::unordered_map<AugmentationPath, std::unordered_set<AugmentationPath, AugmentationPathHash>, AugmentationPathHash> edges; // Adjacency list

    m_graph.paths = paths;
    m_graph.edges = edges;
    m_pcRABAct->Reset();
    m_pcRABSens->Reset();
    m_phase = Construct_Augmentation_Graph;
}
void NeighborhoodGraphController::Reset(){
    BaseConrtoller::Reset();
}

void PrintCByteArray(const argos::CByteArray& cData, UInt8 robotid) {
    std::cout << robotid<<" : ";
    for (size_t i = 0; i < cData.Size(); ++i) {
        std::cout << cData[i] << " ";
    }
    std::cout << std::endl;
}

void AddEdge(std::unordered_set<Edge, EdgeHash>& sEdges, UInt8 uNode1, UInt8 uNode2, bool isMatched, UInt8 weight) {
    // Normalize the edge as (min(uNode1, uNode2), max(uNode1, uNode2))
    sEdges.emplace(std::min(uNode1, uNode2), std::max(uNode1, uNode2), isMatched, weight);
}

void NeighborhoodGraphController::ExchangeNeighborhood() {
    if(m_uCurrentHop >= 1){
        // Receive and merge graphs from neighbors
        const auto& tMessages = m_pcRABSens->GetReadings();
        for (const auto& tMessage : tMessages) {
            UInt8 senderId;
            std::unordered_set<UInt8> sReceivedNodes;
            std::unordered_set<Edge, EdgeHash> sReceivedEdges;

            // Deserialize the received graph
            DeserializeGraph(tMessage.Data, senderId, sReceivedNodes, sReceivedEdges);
            if(senderId != 0){
            // if(m_uRobotId == 15){
            //     cout << senderId ;
            // }
            // Merge the received graph
            m_sNodes.insert(sReceivedNodes.begin(), sReceivedNodes.end());
            for (const auto& edge : sReceivedEdges) {
                AddEdge(m_sEdges, edge.node1, edge.node2, edge.isMatching, edge.weight);
            }

            if (m_uCurrentHop == 1 && m_matched_robot_indexes.find(senderId) == m_matched_robot_indexes.end()) {
                // Add an edge between the current robot and the sender
                UInt8 range= static_cast<UInt8>(tMessage.Range);
                AddEdge(m_sEdges, m_uRobotId, senderId, senderId == m_matchedRobotId, 2*m_visRange - range);
            }
            }
        }
    }
    // Serialize the graph into a message
    CByteArray cData;
    SerializeGraph(cData);
    m_pcRABAct->SetData(cData);
}

void NeighborhoodGraphController::SerializeGraph(CByteArray& cData) {
    // Serialize nodes
    cData << m_uRobotId;
    UInt8 num_nodes = static_cast<UInt8>(m_sNodes.size());
    cData << num_nodes;
    for (UInt8 uNode : m_sNodes) {
        cData << uNode;
    }

    // Serialize edges
    cData << static_cast<UInt8>(m_sEdges.size());
    for (const auto& edge : m_sEdges) {
        UInt8 matching = 0;
        if(edge.isMatching){
            matching = 1;
        }
        cData << edge.node1 << edge.node2 << matching << edge.weight;
    }
    for(int i = cData.Size(); i < 1000; i++){
        UInt8 a = 0;
        cData << a;
    }
}

void NeighborhoodGraphController::DeserializeGraph(const CByteArray& cData,
                        UInt8& senderId,
                        std::unordered_set<UInt8>& sNodes,
                        std::unordered_set<Edge, EdgeHash>& sEdges) {
    // if(m_uRobotId == 15){
    //     cout << m_uRobotId << " data:";
    //     for(int i = 0; i < 30; i++){
    //         cout << cData[i] << " ";
    //     }
    //     cout << endl;
    // }

    size_t index = 0;
    senderId = cData[index++];
    // Deserialize nodes
    UInt8 uNodeCount = cData[index++];
    for (UInt8 i = 0; i < uNodeCount; ++i) {
        if(m_matched_robot_indexes.find(cData[index]) == m_matched_robot_indexes.end()){
            sNodes.insert(cData[index]);
        }
        index++;
    }
    // Deserialize edges
    UInt8 uEdgeCount = cData[index++];
    for (UInt8 i = 0; i < uEdgeCount; ++i) {
        UInt8 uNode1 = cData[index++];
        UInt8 uNode2 = cData[index++];
        UInt8 isMatching = cData[index++];
        UInt8 weight = cData[index++];
        AddEdge(sEdges, uNode1, uNode2, isMatching == 1, weight);
    }
}

Real GaziRepultion3(double distance){
    if(distance > 20){
        return 0;
    }
    double b = 7;
    double c = 15;
    double v = distance * (- b * ::pow(M_E, -::pow(distance,2)/c));
    return v;
}
CVector2 NeighborhoodGraphController::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 local_repulsion_force;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            // cout << tMsgs[i].Data[1] << endl;
            if(tMsgs[i].Data[1] != m_matchedRobotId){
                Real fLJ = GaziRepultion3(tMsgs[i].Range);
                local_repulsion_force += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
        }
        // cout << local_repulsion_force << endl;
        return local_repulsion_force;
    }
    else {
        return CVector2();
    }
}


void NeighborhoodGraphController::PropegratePaths(std::vector<std::vector<UInt8>>& paths){
    // CByteArray cData;
    // SerializePaths(paths, cData);
    // m_pcRABAct->SetData(cData);

    // // Receive and integrate paths from neighbors
    // const auto& tMessages = m_pcRABSens->GetReadings();
    // for (const auto& tMessage : tMessages) {
    //     std::unordered_map<UInt8, PathNode> receivedPaths;
    //     DeserializePaths(tMessage.Data, receivedPaths);
    //     for (const auto& [id, pathNode] : receivedPaths) {
    //         localPaths[id] = pathNode; // Merge received paths
    //     }
    // }
}


void NeighborhoodGraphController::FinalizeGraph() {
    // Output or process the final graph (for debugging purposes here)
    cout << "Robot " << m_uRobotId << " final graph:\n";
    cout << "Nodes: ";
    for (UInt8 node : m_sNodes) {
        cout << node << " ";
    }
    cout << "\nEdges: ";
    for (const auto& edge : m_sEdges) {
        cout << "(" << edge.node1 << ", " << edge.node2 << "," <<edge.isMatching << ") ";
    }
    cout << "\n";
}



void NeighborhoodGraphController::AddHopToGraph(){
    std::unordered_set<Edge, EdgeHash> edges_to_add;
    for(const auto& edge1 : m_sEdges){
        for(const auto& edge2 : m_sEdges){
            if(edge1 == edge2 || edge1.weight == 0 || edge2.weight == 0) continue;
            if(edge1.node1 == edge2.node1 && edge1.node2 < edge2.node2){
                AddEdge(edges_to_add,edge1.node2, edge2.node2, false, 0);
            }
            else if(edge1.node1 == edge2.node2 && edge1.node2 < edge2.node1){
                AddEdge(edges_to_add,edge1.node2, edge2.node1, false, 0);
            }
            else if(edge1.node2 == edge2.node2 && edge1.node1 < edge2.node1){
                AddEdge(edges_to_add,edge1.node1, edge2.node1, false, 0);
            }
        }   
    }
    m_sEdges.insert(edges_to_add.begin(), edges_to_add.end());
}

void NeighborhoodGraphController::FindAugmentationPaths(UInt8 startNode, UInt8 maxLength,
                                                         std::vector<std::vector<UInt8>>& paths,
                                                         std::unordered_map<size_t, Real>& pathGains) {
    // BFS-based approach to find augmentation paths
    std::queue<std::tuple<std::vector<UInt8>, bool, Real>> queue; // {path, isLastEdgeMatched, currentGain}
    queue.push({{startNode}, false, 0.0}); // Start with the initial node, unmatched state, and zero gain.
    size_t pathIndex = 0;
    while (!queue.empty()) {
        auto [currentPath, isLastEdgeMatched, currentGain] = queue.front();
        queue.pop();

        UInt8 currentNode = currentPath.back();
        if (currentPath.size() > 1 && currentNode == startNode && currentPath.size() - 1 <= maxLength) {
            // Augmentation path found (excluding cycles)
            paths.push_back(currentPath);
            pathGains[pathIndex++] = currentGain;
            continue;
        }
        else if (currentPath.size() > 1 && !isLastEdgeMatched){
            paths.push_back(currentPath);
            pathGains[pathIndex++] = currentGain;
        }

        if (currentPath.size() > maxLength + 1) {
            continue; // Path exceeds maximum allowed length
        }

        // Explore neighbors
        for (const auto& edge : m_sEdges) {
            UInt8 neighbor = 0;
            if (edge.node1 == currentNode) neighbor = edge.node2;
            else if (edge.node2 == currentNode) neighbor = edge.node1;
            else continue;

            // Check alternation
            if ((currentPath.size() != 1 && edge.isMatching != isLastEdgeMatched) || (currentPath.size() == 1 && !edge.isMatching)) {
                // Avoid revisiting nodes
                if (std::find(currentPath.begin(), currentPath.end(), neighbor) == currentPath.end()) {
                    std::vector<UInt8> newPath = currentPath;
                    Real newGain = currentGain;
                    // for (const auto& edge2 : m_sEdges) {
                    //     if(edge2.node1 == neighbor && edge2.node2 != )
                    // }
                    if (!edge.isMatching) {
                        newGain += edge.weight;
                    }
                    newPath.push_back(neighbor);
                    queue.push({newPath, edge.isMatching, newGain});
                }
            }
        }
    }
}





REGISTER_CONTROLLER(NeighborhoodGraphController, "neighborhood_graph_controller")
