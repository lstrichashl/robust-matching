#ifndef REGULAR_BOT_H
#define REGULAR_BOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include "src/controllers/abstract_controllers/base_controller.h"

using namespace argos;

class CVirtualForces: public BaseConrtoller {

public:
    CVirtualForces();
    virtual ~CVirtualForces() {}
    virtual void ControlStep();
    virtual void Alone();
    virtual void Paired();
    virtual bool ShouldTransitionToPaired();
    virtual bool ShouldTransitionToAlone();
    virtual CVector2 FlockingVector();


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

};
#endif