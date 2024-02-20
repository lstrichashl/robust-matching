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
    virtual bool ShouldTransitionToAlone();
    virtual CVector2 FlockingVector();
    virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);


   virtual Real LennardJonesPotential(double distance){
      double epsilon = 100;
      double theta = 5;
      return  -epsilon * (::pow(theta/distance, 8) - ::pow(theta/distance, 4));
   }

   virtual Real RepulsiveObstacle(double distance){
      double epsilon = 0.001;
      if(distance < 20) {
         return -epsilon * ::pow(distance, -2);
      }
      return 0;
   }

   virtual Real ElectricalForce(double distance){
      double epsilon = 10;
      return epsilon * ::pow(distance, -2);
   }

   virtual Real LinearForce(double distance){
      double epsilon = 0.5;
      return epsilon * ::pow(distance, 1);
   }

   virtual Real GaziForce(double distance){
      double a = 0.05;
      double b = 20;
      double c = 5;
      double v = distance * (a - b * ::pow(M_E, -::pow(distance,2)/c));
      return v/10000;
   }

   virtual Real Constant(double distance){
      return 1;
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
    CCI_DifferentialSteeringActuator *m_pcWheels;
    CCI_EPuck2LEDsActuator *m_pcLedAct;
    CCI_RangeAndBearingActuator* m_pcRABAct;
    CCI_RangeAndBearingSensor* m_pcRABSens;
    SWheelTurningParams m_sWheelTurningParams;
};
#endif