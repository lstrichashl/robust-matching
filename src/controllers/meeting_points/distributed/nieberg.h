#ifndef nieberg_controller_h
#define nieberg_controller_h


#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include "src/controllers/abstract_controllers/base_controller.h"

using namespace argos;
using namespace std;


struct Edge {
    UInt8 node1, node2;
    bool isMatching;

    // Constructor
    Edge(UInt8 n1, UInt8 n2, bool match) : node1(n1), node2(n2), isMatching(match) {}

    // Equality operator
    bool operator==(const Edge& other) const {
        return (node1 == other.node1 && node2 == other.node2) ||
               (node1 == other.node2 && node2 == other.node1); // Undirected graph
    }
};

// Hash function for Edge
struct edge_hash {
    std::size_t operator()(const Edge& edge) const {
        auto hash1 = std::hash<int>{}(edge.node1);
        auto hash2 = std::hash<int>{}(edge.node2);
        return hash1 ^ hash2;
    }
};


class NeighborhoodGraphController : public BaseConrtoller {
public:
    // Actuators and Sensors
    CCI_RangeAndBearingActuator* m_pcRABActuator;
    CCI_RangeAndBearingSensor* m_pcRABSensor;

    // Parameters
    UInt8 m_uRobotId;                          // Unique ID for the robot
    UInt8 m_uMaxDistance;                      // Maximum neighborhood distance (in hops)
    UInt8 m_uCurrentHop;                       // Current communication hop
    UInt8 m_matchedRobotId;

    // Graph representation
    std::unordered_set<UInt8> m_sNodes;        // Nodes in the graph
    std::unordered_set<Edge, edge_hash> m_sEdges; // Edges in the graph

    NeighborhoodGraphController();
    virtual ~NeighborhoodGraphController() {}
    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset() ;
    virtual void Destroy() override {}

    virtual void ExchangeNeighborhood();

    virtual void SerializeGraph(CByteArray& cData);

    virtual void DeserializeGraph(const CByteArray& cData,
                          UInt8& senderId,
                          std::unordered_set<UInt8>& sNodes,
                          std::unordered_set<Edge, edge_hash>& sEdges);

    virtual void FinalizeGraph();

    virtual void FindAugmentationPaths(UInt8 startNode, UInt8 maxLength,
                                        std::vector<std::vector<UInt8>>& paths);
};

#endif