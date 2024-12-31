#include "crash_controller.h"


void CCrashController::Reset(){
    BaseConrtoller::Reset();
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
    m_heading = CVector2::ZERO;
    fault_type = crash;
}

void CCrashController::ControlStep(){
    m_heading = CVector2::ZERO;
    BaseConrtoller::ControlStep();
}

REGISTER_CONTROLLER(CCrashController, "crash_controller")
