#ifndef ROBUST_MATCHING_H
#define ROBUST_MATCHING_H

#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include "src/controllers/abstract_controllers/base_controller.h"


using namespace argos;

class CRobustMatching : public BaseConrtoller {

public:
    CRobustMatching();
    virtual ~CRobustMatching() {}
    virtual void ControlStep();

};
class CRobustMatchingCrash : public CRobustMatching {

public:
    CRobustMatchingCrash():CRobustMatching(){
        m_typename = "crash";
    }
    void Init(TConfigurationNode& t_node){
        try{
            TConfigurationNode& crashnode = GetNode(t_node, "crash");
            GetNodeAttribute(crashnode, "start_time", m_crash_starttime);
            GetNodeAttribute(crashnode, "end_time", m_crash_endtime);
        }
        catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller crash parameters.", ex);
        }
        CRobustMatching::Init(t_node);
    }
};

#endif
