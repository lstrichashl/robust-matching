#include "meeting_points_epuck.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include "math.h"


CMeetingPointEpuck::CMeetingPointEpuck():
   BaseConrtoller(){
   m_typename = "non_faulty";
   PAIRING_THRESHOLD = 0.07;
   NewIteration();

    m_iPreviousEncoder = 32768;
}

void CMeetingPointEpuck::ControlStep() {
    // m_heading = m_heading.Normalize();
    if(ShouldTransitionToPaired()){
        m_eState = STATE_PAIRED;
        cout << "paired" << endl;
    }
    else if(ShouldTransitionToAlone()){
        m_eState = STATE_ALONE;
    }
    m_pcRABAct->SetData(0, m_eState);

    if(m_eState == STATE_ALONE){
        if(m_meeting_point_phase == Rotate){
            RotateToAngle();
        }
        else if(m_meeting_point_phase == Walk){
            MoveForward();
        }
        else if(m_meeting_point_phase == Done){
            // cout << "done" << endl;
        }
    }
    else{
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }
}

int CMeetingPointEpuck::GetEncoderDiff(){
    const CCI_EPuck2EncoderSensor::SReading& tEncoderReads = m_pcEncoderSensor->GetReadings();
    // cout << "Est. Distance (mm): " << std::fixed << std::setprecision(1) << m_fDistance << std::endl;
    int iDiff = 0;
    int iEnc = tEncoderReads.EncoderRightWheel + 32768;
    iDiff = iEnc - m_iPreviousEncoder;
    if(iDiff > 1000){
        iDiff = iDiff - 32768 * 2;
    }
    else if(iDiff < -1000){
        iDiff = iDiff + 32768 * 2;
    }
    m_iPreviousEncoder = iEnc;
    return iDiff;
}

void CMeetingPointEpuck::MoveForward(){
    int iDiff = GetEncoderDiff();
    m_fDistance += float(iDiff) * 2 * CRadians::PI.GetValue() * 0.0205f;
    if(m_fDistance < (m_heading.Length()-PAIRING_THRESHOLD) * 500){
        m_pcWheels->SetLinearVelocity(5.0f, 5.0f);
    }
    else{
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        m_meeting_point_phase = Done;
    }
}

void CMeetingPointEpuck::RotateToAngle(){
    int iDiff = GetEncoderDiff();
    
    m_relative_orientation += float(iDiff) * 2 * CRadians::PI.GetValue() / 1292.7f;
    if(m_relative_orientation > 2 * CRadians::PI.GetValue()){
        m_relative_orientation -= 2 * CRadians::PI.GetValue();
    }
    Real angle = m_heading.Angle().UnsignedNormalize().GetValue();
    Real error = m_relative_orientation-angle;

    if(error > CRadians::PI.GetValue()){
        error -= 2 * CRadians::PI.GetValue();
    }
    if(error < -CRadians::PI.GetValue()){
        error += 2 * CRadians::PI.GetValue();
    }

    if(error < -0.02){
        m_pcWheels->SetLinearVelocity(-1.0f, 1.0f);
    }
    else if(error > 0.02){
        m_pcWheels->SetLinearVelocity(1.0f, -1.0f);
    }
    else{
        m_pcWheels->SetLinearVelocity(0.0f,0.0f);
        m_meeting_point_phase = Walk;
    }
}

void CMeetingPointEpuck::NewIteration(){
    m_meeting_point_phase = Rotate;
    m_fDistance = 0.0;
    m_relative_orientation = 0;
}




REGISTER_CONTROLLER(CMeetingPointEpuck, "meeting_point_epuck_controller")
