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


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(std::min(p.first, p.second));
        auto h2 = std::hash<T2>{}(std::max(p.first, p.second));
        return h1 ^ h2; // Combine hash values (XOR is simple and effective here)
    }
};

// Ensure two pairs are equal if their normalized forms are equal
bool operator==(const std::pair<UInt8, UInt8>& lhs, const std::pair<UInt8, UInt8>& rhs) {
    return (std::min(lhs.first, lhs.second) == std::min(rhs.first, rhs.second)) &&
           (std::max(lhs.first, lhs.second) == std::max(rhs.first, rhs.second));
}

class NeighborhoodGraphController : public BaseConrtoller {
public:
    // Actuators and Sensors
    CCI_RangeAndBearingActuator* m_pcRABActuator;
    CCI_RangeAndBearingSensor* m_pcRABSensor;

    // Parameters
    UInt8 m_uRobotId;                          // Unique ID for the robot
    UInt8 m_uMaxDistance;                      // Maximum neighborhood distance (in hops)
    UInt8 m_uCurrentHop;                       // Current communication hop

    // Graph representation
    std::unordered_set<UInt8> m_sNodes;        // Nodes in the graph
    std::unordered_set<std::pair<UInt8, UInt8>, pair_hash> m_sEdges; // Edges in the graph

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
                          std::unordered_set<std::pair<UInt8, UInt8>, pair_hash>& sEdges);

    virtual void FinalizeGraph();
};

#endif