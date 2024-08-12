#ifndef ROBUST_MATCHING_H
#define ROBUST_MATCHING_H

#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include "src/controllers/abstract_controllers/base_controller.h"
#include <set>

using namespace argos;

class CMeetingPoint : public BaseConrtoller {

public:
    CMeetingPoint();
    virtual ~CMeetingPoint() {}
    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset();

    CVector3 m_target_position;
    std::set<int> m_matched_robot_ids;

protected:
    CCI_QuadRotorPositionActuator* m_pcPosAct;
    CCI_PositioningSensor* m_pcPosSens;
};


class CMeetingPointCrash : public CMeetingPoint {
public:
    CMeetingPointCrash();
    virtual ~CMeetingPointCrash() {}
    virtual void ControlStep();
};

#endif
