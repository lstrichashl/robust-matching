#ifndef REGULAR_BOT_H
#define REGULAR_BOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include "src/controllers/abstract_controllers/base_controller.h"

using namespace argos;

class CTripletForces: public BaseConrtoller {

public:
    CTripletForces();
    virtual ~CTripletForces() {}
    virtual void ControlStep();
    virtual CVector2 FlockingVector();
    virtual EState UpdateEState();

   virtual Real RepulsiveObstacle(double distance){
      double epsilon = 0.001;
      if(distance < 20) {
         return -epsilon * ::pow(distance, -2);
      }
      return 0;
   }

   virtual Real ElectricalForce(double distance){
      double epsilon = 1;
      return epsilon * ::pow(distance/30, -2);
   }

   virtual Real LinearForce(double distance){
      double epsilon = 0.5;
      return epsilon * ::pow(distance, 1);
   }

   virtual Real GaziForce(double distance){
      return GaziAttraction(distance) + GaziRepultion(distance);
   }

   virtual Real GaziRepultion(double distance){
      if(distance > 20){
         return 0;
      }
      double b = 7;
      double c = 15;
      double v = distance * (- b * ::pow(M_E, -::pow(distance,2)/c));
      return v;
   }

   virtual Real GaziAttraction(double distance){
      double a = 0.01;
      return distance * a;
   }

   virtual Real Constant(double distance){
      return 1;
   }

protected:
   UInt8 number_of_robots_in_group;

};
#endif