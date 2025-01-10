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
    UInt8 node1;
    UInt8 node2;
    bool isMatching;
    UInt8 weight; // Weight of the edge

    Edge(UInt8 n1, UInt8 n2, bool match, UInt8 w)
        : node1(n1), node2(n2), isMatching(match), weight(w) {}

    bool operator==(const Edge& other) const {
        return (node1 == other.node1 && node2 == other.node2) ||
               (node1 == other.node2 && node2 == other.node1);
    }
};

struct EdgeHash {
    std::size_t operator()(const Edge& e) const {
        return std::hash<UInt8>()(e.node1) ^ std::hash<UInt8>()(e.node2);
    }
};

struct AugmentationPath {
    std::vector<UInt8> nodes;  // Nodes in the augmentation path
    Real gain;                 // Gain associated with the path

    bool operator==(const AugmentationPath& other) const {
        if (nodes.size() != other.nodes.size()) {
            return false;
        }

        if (nodes == other.nodes) {
            return true;
        }

        return std::equal(nodes.begin(), nodes.end(), other.nodes.rbegin());
    }
};

struct AugmentationPathHash {
    size_t operator()(const AugmentationPath& path) const {
        std::vector<UInt8> sortedNodes = path.nodes;
        std::sort(sortedNodes.begin(), sortedNodes.end()); //TODO: sort or reverse? sort can remove path that I dont want to?

        size_t hash = 0;
        for (UInt8 node : sortedNodes) {
            hash ^= std::hash<UInt8>()(node) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

struct AugmentationGraph {
    std::unordered_set<AugmentationPath, AugmentationPathHash> paths; // Unique paths
    std::unordered_map<AugmentationPath, std::unordered_set<AugmentationPath, AugmentationPathHash>, AugmentationPathHash> edges; // Adjacency list
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
    std::unordered_set<Edge, EdgeHash> m_sEdges; // Edges in the graph

    std::vector<std::vector<UInt8>> m_vAugmentationPaths;
    std::unordered_map<size_t, Real> m_vPathGains;

    AugmentationGraph m_graph;

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
                          std::unordered_set<Edge, EdgeHash>& sEdges);

    virtual void FinalizeGraph();

    virtual void FindAugmentationPaths(UInt8 startNode, 
                                        UInt8 maxLength, 
                                        std::vector<std::vector<UInt8>>& paths, 
                                        std::unordered_map<size_t, Real>& pathGains);

    virtual void AddHopToGraph();
    virtual void StartMatching();

    virtual AugmentationGraph GetAugmentationGraph(){
        return m_graph;
    }
};

#endif