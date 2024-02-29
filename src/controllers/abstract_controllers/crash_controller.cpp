#include "crash_controller.h"


void CCrashController::Reset(){
    BaseConrtoller::Reset();
    m_eState = STATE_ALONE;
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
    m_pcRABAct->SetData(0, m_eState);
}

void CCrashController::ControlStep(){
    if(ShouldTransitionToPaired()){
        m_eState = STATE_PAIRED;
    }
    else if(ShouldTransitionToAlone()){
        m_eState = STATE_ALONE;
    }
    m_pcRABAct->SetData(0, m_eState);
}

REGISTER_CONTROLLER(CCrashController, "crash_controller")
