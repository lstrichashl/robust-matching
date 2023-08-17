#ifndef MATCHING_LOOP_FUNCTIONS_H
#define MATCHING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>


using namespace argos;

class CMatchingLoopFunctions : public CLoopFunctions {

public:
   CMatchingLoopFunctions();
   virtual ~CMatchingLoopFunctions() {}
   virtual void PreStep();

};

#endif
