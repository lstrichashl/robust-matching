#include "crash_controller.h"


void CCrashController::Reset(){
    BaseConrtoller::Reset();
    m_eState = STATE_ALONE;
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
    m_pcRABAct->SetData(0, m_eState);
    m_heading = CVector2::ZERO;
}

void CCrashController::ControlStep(){
    BaseConrtoller::ControlStep();
    m_heading = CVector2::ZERO;
}

REGISTER_CONTROLLER(CCrashController, "crash_controller")
