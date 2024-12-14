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

    m_iPreviousEncoderRight = 32768;
    m_iPreviousEncoderLeft = 32768;
    CVector2 initial_pos(0,0);
    m_encoder_position = initial_pos;
}


CVector2 CMeetingPointEpuck::ToMateVector(CVector2 self_position, CVector2 meeting_point) {
    CVector2 to_mate = meeting_point - self_position;
    CRadians cZAngle(m_relative_orientation);
    return to_mate.Rotate(-cZAngle);
}

void CMeetingPointEpuck::ControlStep() {
    // if(ShouldTransitionToPaired()){
    //     m_eState = STATE_PAIRED;
    //     cout << "paired" << endl;
    // }
    // else if(ShouldTransitionToAlone()){
    //     m_eState = STATE_ALONE;
    // }
    // m_pcRABAct->SetData(0, m_eState);

    // if(m_eState == STATE_ALONE){
    //     if(m_meeting_point_phase == Rotate){
    //         RotateToAngle();
    //     }
    //     else if(m_meeting_point_phase == Walk){
    //         MoveForward();
    //     }
    //     else if(m_meeting_point_phase == Done){
    //         // cout << "done" << endl;
    //     }
    // }
    // else{
    //     m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    // }

    int diffRight = GetEncoderDiffRight();
    int diffLeft = GetEncoderDiffLeft();

    Real orientationChange = float(diffRight-diffLeft) * CRadians::PI.GetValue() / 1292.7f;
    m_relative_orientation += orientationChange;
    if(m_relative_orientation > CRadians::PI.GetValue()){
        m_relative_orientation -= 2 * CRadians::PI.GetValue();
    }
    if(m_relative_orientation < -CRadians::PI.GetValue()){
        m_relative_orientation += 2 * CRadians::PI.GetValue();
    }


    Real distanceChange = float(diffRight+diffLeft) * CRadians::PI.GetValue() * 0.0205f;
    m_fDistance += distanceChange;
    

    CVector2 changevector(-distanceChange/1000, CRadians(m_relative_orientation));
    m_encoder_position -= changevector;
    // m_heading = ToMateVector(m_encoder_position,m_meeting_point);
    // cout << m_heading << endl;
    if(m_heading.Length()-PAIRING_THRESHOLD/2 < 0) {
        m_heading = CVector2::ZERO;
    }
    else{
        m_heading = m_heading.Normalize() * 5;
    }
    m_heading += FlockingVector();
    BaseConrtoller::ControlStep();
}

int CMeetingPointEpuck::GetEncoderDiffRight(){
    const CCI_EPuck2EncoderSensor::SReading& tEncoderReads = m_pcEncoderSensor->GetReadings();
    // cout << "Est. Distance (mm): " << std::fixed << std::setprecision(1) << m_fDistance << std::endl;
    int iDiff = 0;
    int iEnc = tEncoderReads.EncoderRightWheel + 32768;
    iDiff = iEnc - m_iPreviousEncoderRight;
    if(iDiff > 1000){
        iDiff = iDiff - 32768 * 2;
    }
    else if(iDiff < -1000){
        iDiff = iDiff + 32768 * 2;
    }
    m_iPreviousEncoderRight = iEnc;
    return iDiff;
}
int CMeetingPointEpuck::GetEncoderDiffLeft(){
    const CCI_EPuck2EncoderSensor::SReading& tEncoderReads = m_pcEncoderSensor->GetReadings();
    // cout << "Est. Distance (mm): " << std::fixed << std::setprecision(1) << m_fDistance << std::endl;
    int iDiff = 0;
    int iEnc = tEncoderReads.EncoderLeftWheel + 32768;
    iDiff = iEnc - m_iPreviousEncoderLeft;
    if(iDiff > 1000){
        iDiff = iDiff - 32768 * 2;
    }
    else if(iDiff < -1000){
        iDiff = iDiff + 32768 * 2;
    }
    m_iPreviousEncoderLeft = iEnc;
    return iDiff;
}

CVector2 CMeetingPointEpuck::GetDirectionFromEncoders(){
    int diffRight = GetEncoderDiffRight();
    int diffLeft = GetEncoderDiffLeft();

    m_relative_orientation += float(diffRight-diffLeft) * 2 * CRadians::PI.GetValue() / 1292.7f;
    m_fDistance += float(diffRight+diffLeft) * 2 * CRadians::PI.GetValue() * 0.0205f;

}

void CMeetingPointEpuck::MoveForward(){
    int iDiff = GetEncoderDiffRight();
    m_fDistance += float(iDiff) * 2 * CRadians::PI.GetValue() * 0.0205f;
    if(m_fDistance < (m_meeting_point.Length()-PAIRING_THRESHOLD/2) * 1000){
        m_pcWheels->SetLinearVelocity(5.0f, 5.0f);
    }
    else{
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        m_meeting_point_phase = Done;
    }
}

void CMeetingPointEpuck::RotateToAngle(){
    int iDiff = GetEncoderDiffRight();
    
    m_relative_orientation += float(iDiff) * 2 * CRadians::PI.GetValue() / 1292.7f;
    if(m_relative_orientation > 2 * CRadians::PI.GetValue()){
        m_relative_orientation -= 2 * CRadians::PI.GetValue();
    }
    Real angle = m_meeting_point.Angle().UnsignedNormalize().GetValue();
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


Real GaziRepultion(double distance){
    if(distance > 20){
        return 0;
    }
    double b = 5;
    double c = 20;
    double v = distance * (- b * ::pow(M_E, -::pow(distance,2)/c));
    return v;
}

CVector2 CMeetingPointEpuck::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 local_repulsion_force;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            // cout << tMsgs[i].Data[1] << endl;
            if(tMsgs[i].Data[1] != stoi(matched_robot_id)){
                Real fLJ = GaziRepultion(tMsgs[i].Range);
                local_repulsion_force += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
        }
        // cout << local_repulsion_force << endl;
        return local_repulsion_force;
    }
    else {
    // cout << GetId()<< ":"  << matched_robot_id << endl;
        return CVector2();
    }
}


REGISTER_CONTROLLER(CMeetingPointEpuck, "meeting_point_epuck_controller")
