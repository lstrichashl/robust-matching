#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <iostream>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_encoder_sensor.h>
#include <set>

using namespace std;
using namespace argos;

enum EState {
   STATE_ALONE = 0,
   STATE_PAIRED
};

class BaseConrtoller: public CCI_Controller{

public:
   BaseConrtoller();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);
   virtual bool ShouldTransitionToPaired();
   virtual bool ShouldTransitionToAlone();
   virtual void ControlStep();
   virtual std::string GetType(){
      return m_typename;
   }
   virtual EState GetEState(){
      return m_eState;
   }
   double PAIRING_THRESHOLD = 0.07;
   CVector2 m_heading;
   UInt32 m_time;
   CVector2 m_position;
   CQuaternion m_orientation;
   bool m_is_crash;
   int m_crash_starttime = 100000000;
   int m_crash_endtime = 100000000;
   int m_crash_time;
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
   virtual CVector2 RandomWalk();
   std::set<int> m_matched_robot_ids;
   virtual void NewIteration() {}

protected:
    string m_typename;
    EState m_eState;
    CCI_DifferentialSteeringActuator *m_pcWheels;
    CCI_EPuck2LEDsActuator *m_pcLedAct;
    CCI_RangeAndBearingActuator* m_pcRABAct;
    CCI_RangeAndBearingSensor* m_pcRABSens;
    CCI_EPuck2EncoderSensor* m_pcEncoderSensor;
    SWheelTurningParams m_sWheelTurningParams;

   
private:
   CVector2 random_destination;
   CRandom::CRNG* pcRNG;
};

#endif