#ifndef REGULAR_BOT_H
#define REGULAR_BOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include "src/controllers/abstract_controllers/base_controller.h"

using namespace argos;

class CVirtualForces: public CCI_Controller, public BaseConrtoller {

public:
    CVirtualForces();
    virtual ~CVirtualForces() {}
    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Alone();
    virtual void Paired();
    virtual bool ShouldTransitionToPaired();
    virtual CVector2 FlockingVector();
    virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);


   virtual Real LennardJonesPotential(double distance){
      double epsilon = 100;
      double theta = 5;
      return  -epsilon * (::pow(theta/distance, 6) - ::pow(theta/distance, 3));
   }

   virtual Real RepulsiveObstacle(double distance){
      double epsilon = 300;
      double beta = 1;
      if(distance < 20) {
         return -epsilon/beta * ::pow(beta/distance, 2);
      }
      return 0;
   }

   virtual Real ElectricalForcePotentail(double distance){
      double epsilon = 100;
      return epsilon * ::pow(distance, -2);
   }



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

protected:
   /* Current robot state */
   enum EState {
      STATE_ALONE = 0,
      STATE_PAIRED
   };

protected:
    EState m_eState;
    CCI_DifferentialSteeringActuator *m_pcWheels;
    CCI_EPuck2LEDsActuator *m_pcLedAct;
    CCI_RangeAndBearingActuator* m_pcRABAct;
    CCI_RangeAndBearingSensor* m_pcRABSens;
    SWheelTurningParams m_sWheelTurningParams;
};
#endif