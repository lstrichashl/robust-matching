#ifndef COMMITED_MATCHING_LOOP_FUNCTIONS_H
#define COMMITED_MATCHING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <vector>
#include "src/loop_functions/matching_loop_functions/matching_loop_functions.h"


using namespace argos;

class CCommitedMatchingLoopFunctions : public CMatchingLoopFunctions {

public:
   CCommitedMatchingLoopFunctions();
   virtual ~CCommitedMatchingLoopFunctions() {}
   virtual void PreStep();


private:
   bool m_isCommited;
};

#endif
