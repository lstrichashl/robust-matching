#ifndef MOVE_TO_POINT_H
#define MOVE_TO_POINT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>


using namespace argos;

class CMoveToPoint : public CCI_Controller {

public:
    CMoveToPoint();
    virtual ~CMoveToPoint() {}
    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void SetWheelSpeedsFromVector(const CVector3& c_heading);
    virtual CRadians GetZAngleOrientation();
    virtual void Reset();
    virtual CVector3 MoveToVector();
    CVector3 move_to_position;

    struct SWheelTurningParams {

      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   virtual std::string GetType(){
      return m_typename;
   }

protected:
    CCI_DifferentialSteeringActuator *m_pcWheels;
    SWheelTurningParams m_sWheelTurningParams;
    CCI_EPuck2LEDsActuator *m_pcLedAct;
    std::string m_typename;
};


#endif
