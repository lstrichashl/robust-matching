#include "nieberg.h"

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

    // Add self to graph
    m_sNodes.insert(m_uRobotId);
}

void NeighborhoodGraphController::ControlStep(){
    if (m_uCurrentHop < m_uMaxDistance) {
        ExchangeNeighborhood();
        m_uCurrentHop++;
    } else {
        // Neighborhood discovery complete; finalize graph
        FinalizeGraph();
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

void AddEdge(std::unordered_set<std::pair<UInt8, UInt8>, pair_hash>& sEdges, UInt8 uNode1, UInt8 uNode2) {
    // Normalize the edge as (min(uNode1, uNode2), max(uNode1, uNode2))
    sEdges.insert(std::make_pair(std::min(uNode1, uNode2), std::max(uNode1, uNode2)));
}

void NeighborhoodGraphController::ExchangeNeighborhood() {
    if(m_uCurrentHop >= 1){
        // Receive and merge graphs from neighbors
        const auto& tMessages = m_pcRABSensor->GetReadings();
        for (const auto& tMessage : tMessages) {
            UInt8 senderId;
            std::unordered_set<UInt8> sReceivedNodes;
            std::unordered_set<std::pair<UInt8, UInt8>, pair_hash> sReceivedEdges;

            // Deserialize the received graph
            DeserializeGraph(tMessage.Data, senderId, sReceivedNodes, sReceivedEdges);

            // Merge the received graph
            m_sNodes.insert(sReceivedNodes.begin(), sReceivedNodes.end());
            for (const auto& edge : sReceivedEdges) {
                AddEdge(m_sEdges, edge.first, edge.second); // Use normalized insertion
            }

            if (m_uCurrentHop == 1) {
                // Add an edge between the current robot and the sender
                AddEdge(m_sEdges, m_uRobotId, senderId);
                std::cout << m_uRobotId << " " << senderId << std::endl;
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
        cData << edge.first << edge.second;
    }
    cout << cData.Size() << endl;
    // while(i < 30){
    //     cData << static_cast<UInt8>(0); // Append a zero
    //     i = cData.Size();
    //     cout << i << endl;
    // }
    for(int i = cData.Size(); i < 50; i++){
        UInt8 a = 0;
        cData << a;
    }

    PrintCByteArray(cData, m_uRobotId);
}

void NeighborhoodGraphController::DeserializeGraph(const CByteArray& cData,
                        UInt8& senderId,
                        std::unordered_set<UInt8>& sNodes,
                        std::unordered_set<std::pair<UInt8, UInt8>, pair_hash>& sEdges) {
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
        AddEdge(sEdges, uNode1, uNode2);
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
        cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    cout << "\n";
}

REGISTER_CONTROLLER(NeighborhoodGraphController, "neighborhood_graph_controller")
