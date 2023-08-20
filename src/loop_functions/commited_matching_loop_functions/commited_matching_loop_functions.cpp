#include "commited_matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/robust_matching.h"
#include <list>

CCommitedMatchingLoopFunctions::CCommitedMatchingLoopFunctions():
    CMatchingLoopFunctions(),
    m_isCommited(false)
    {}

void CCommitedMatchingLoopFunctions::PreStep(){
    if(!m_isCommited){
        CMatchingLoopFunctions::PreStep();
        m_isCommited=true;
    }
}

REGISTER_LOOP_FUNCTIONS(CCommitedMatchingLoopFunctions, "commited_matching_loop_functions")
