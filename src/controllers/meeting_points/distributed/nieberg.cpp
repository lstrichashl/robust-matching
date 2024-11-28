#include "nieberg.h"
#include <queue>

NeighborhoodGraphController::NeighborhoodGraphController(){

}

void NeighborhoodGraphController::Init(TConfigurationNode& t_node) {
    // Initialize actuators and sensors
    m_pcRABActuator = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
    m_pcRABSensor = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );

    // Initialize robot parameters
    GetNodeAttribute(t_node, "max_distance", m_uMaxDistance);

    string id = GetId();
    m_uRobotId = stoi(id);
    m_uCurrentHop = 0;
    m_matchedRobotId = 255;
    if(m_uRobotId == 192){
        m_matchedRobotId = 199;
    }    
    if(m_uRobotId == 199){
        m_matchedRobotId = 192;
    }    
    if(m_uRobotId == 197){
        m_matchedRobotId = 190;
    }
    if(m_uRobotId == 190){
        m_matchedRobotId = 197;
    }    

    // Add self to graph
    m_sNodes.insert(m_uRobotId);
}

void NeighborhoodGraphController::ControlStep(){
    if (m_uCurrentHop < m_uMaxDistance) {
        ExchangeNeighborhood();
        m_uCurrentHop++;
    } else {
        // Neighborhood discovery complete; finalize graph
        // FinalizeGraph();

        std::vector<std::vector<UInt8>> augmentationPaths;
        FindAugmentationPaths(m_uRobotId, m_uMaxDistance, augmentationPaths);

        // Print all augmentation paths
        if(GetId() == "196"){
            std::cout << "Robot " << m_uRobotId << " augmentation paths:\n";
            for (const auto& path : augmentationPaths) {
                std::cout << "Path: ";
                for (UInt8 node : path) {
                    std::cout << node << " ";
                }
                std::cout << std::endl;
            }
        }
    }
}

void NeighborhoodGraphController::Reset(){
    m_sNodes.clear();
    m_sEdges.clear();
    m_sNodes.insert(m_uRobotId);
    m_uCurrentHop = 0;
}

void PrintCByteArray(const argos::CByteArray& cData, UInt8 robotid) {
    std::cout << robotid<<" : ";
    for (size_t i = 0; i < cData.Size(); ++i) {
        std::cout << cData[i] << " ";
    }
    std::cout << std::endl;
}

void AddEdge(std::unordered_set<Edge, edge_hash>& sEdges, UInt8 uNode1, UInt8 uNode2, bool isMatched) {
    // Normalize the edge as (min(uNode1, uNode2), max(uNode1, uNode2))
    sEdges.emplace(std::min(uNode1, uNode2), std::max(uNode1, uNode2), isMatched);
}

void NeighborhoodGraphController::ExchangeNeighborhood() {
    if(m_uCurrentHop >= 1){
        // Receive and merge graphs from neighbors
        const auto& tMessages = m_pcRABSensor->GetReadings();
        for (const auto& tMessage : tMessages) {
            UInt8 senderId;
            std::unordered_set<UInt8> sReceivedNodes;
            std::unordered_set<Edge, edge_hash> sReceivedEdges;

            // Deserialize the received graph
            DeserializeGraph(tMessage.Data, senderId, sReceivedNodes, sReceivedEdges);

            // Merge the received graph
            m_sNodes.insert(sReceivedNodes.begin(), sReceivedNodes.end());
            for (const auto& edge : sReceivedEdges) {
                AddEdge(m_sEdges, edge.node1, edge.node2, edge.isMatching); // Use normalized insertion
            }

            if (m_uCurrentHop == 1) {
                // Add an edge between the current robot and the sender
                AddEdge(m_sEdges, m_uRobotId, senderId, senderId == m_matchedRobotId);
            }
        }
    }
    // Serialize the graph into a message
    CByteArray cData;
    SerializeGraph(cData);
    m_pcRABActuator->SetData(cData);
}

void NeighborhoodGraphController::SerializeGraph(CByteArray& cData) {
    // Serialize nodes
    cData << m_uRobotId;
    cData << static_cast<UInt8>(m_sNodes.size());
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
        cData << edge.node1 << edge.node2 << matching;
    }
    for(int i = cData.Size(); i < 500; i++){
        UInt8 a = 0;
        cData << a;
    }

    // PrintCByteArray(cData, m_uRobotId);
}

void NeighborhoodGraphController::DeserializeGraph(const CByteArray& cData,
                        UInt8& senderId,
                        std::unordered_set<UInt8>& sNodes,
                        std::unordered_set<Edge, edge_hash>& sEdges) {
    size_t index = 0;
    senderId = cData[index++];
    // Deserialize nodes
    UInt8 uNodeCount = cData[index++];
    for (UInt8 i = 0; i < uNodeCount; ++i) {
        sNodes.insert(cData[index++]);
    }

    // Deserialize edges
    UInt8 uEdgeCount = cData[index++];
    for (UInt8 i = 0; i < uEdgeCount; ++i) {
        UInt8 uNode1 = cData[index++];
        UInt8 uNode2 = cData[index++];
        UInt8 isMatching = cData[index++];
        AddEdge(sEdges, uNode1, uNode2, isMatching == 1);
    }
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
        cout << "(" << edge.node1 << ", " << edge.node2 << ") ";
    }
    cout << "\n";
}




void NeighborhoodGraphController::FindAugmentationPaths(UInt8 startNode, UInt8 maxLength,
                                                         std::vector<std::vector<UInt8>>& paths) {
    // BFS-based approach to find augmentation paths
    std::queue<std::pair<std::vector<UInt8>, bool>> queue; // {path, isLastEdgeMatched}
    queue.push({{startNode}, false}); // Start with the unmatched node

    while (!queue.empty()) {
        auto [currentPath, isLastEdgeMatched] = queue.front();
        queue.pop();

        UInt8 currentNode = currentPath.back();
        if (currentPath.size() > 1 && currentNode == startNode && currentPath.size() - 1 <= maxLength) {
            // Augmentation path found (excluding cycles)
            paths.push_back(currentPath);
            continue;
        }
        else if (currentPath.size() > 1 && !isLastEdgeMatched){
            paths.push_back(currentPath);
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

            // if(GetId() == "196"){
            //     for(const auto node: currentPath){
            //         cout << node << " ";
            //     }
            //     cout << endl;
            // }
            // Check alternation
            if (edge.isMatching != isLastEdgeMatched || (currentPath.size() == 1 && !edge.isMatching)) {
                // Avoid revisiting nodes
                if (std::find(currentPath.begin(), currentPath.end(), neighbor) == currentPath.end()) {
                    std::vector<UInt8> newPath = currentPath;
                    newPath.push_back(neighbor);
                    queue.push({newPath, edge.isMatching});
                }
            }
        }
    }
}





REGISTER_CONTROLLER(NeighborhoodGraphController, "neighborhood_graph_controller")
