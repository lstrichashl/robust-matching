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
   STATE_PAIRED,
   STATE_TRIPLET
};

enum FaultyType{
   nonfaulty = 0,
   crash = 1,
   keep_distance = 2,
   virtual_forces_walk_away = 3
};

class BaseConrtoller: public CCI_Controller{

public:
   BaseConrtoller();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);
   virtual bool ShouldTransitionToPaired();
   virtual bool ShouldTransitionToAlone();
   virtual CVector2 KeepDistanceFlockingVector();
   virtual CVector2 VirtualForceWalkAwayFlockingVector();
   virtual void ControlStep();
   virtual EState GetEState(){
      return m_eState;
   }
   virtual string GetId();
   double PAIRING_THRESHOLD = 0.07;
   CVector2 m_heading;
   UInt32 m_time;
   CVector2 m_position;
   CQuaternion m_orientation;
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
   std::set<int> m_matched_robot_indexes;
   std::set<int> m_rejected_sin_robot_ids;
   string matched_robot_id;
   virtual void NewIteration() {}
   CVector2 m_meeting_point;
   int time_for_wait_for_parter_in_target = 0;

   FaultyType fault_type;
   
   virtual Real LennardJonesPotential(double distance){
      double epsilon = 10;
      double theta = 15;
      return  -epsilon * (::pow(theta/distance, 4) - ::pow(theta/distance, 2));
   }
   virtual Real ElectricalForce(double distance){
      double epsilon = 1;
      return epsilon * ::pow(distance/30, -2);
   }
protected:
    EState m_eState;
    CCI_DifferentialSteeringActuator *m_pcWheels;
    CCI_EPuck2LEDsActuator *m_pcLedAct;
    CCI_RangeAndBearingActuator* m_pcRABAct;
    CCI_RangeAndBearingSensor* m_pcRABSens;
    CCI_EPuck2EncoderSensor* m_pcEncoderSensor;
    SWheelTurningParams m_sWheelTurningParams;
    string m_new_id;

   
protected:
   CVector2 random_destination;
   CRandom::CRNG* pcRNG;
};


string toStringFaultType(FaultyType type);
#endif