#include "base_controller.h"


BaseConrtoller::BaseConrtoller():
   m_pcWheels(NULL),
   m_pcLedAct(NULL) {
      pcRNG = CRandom::CreateRNG("argos");
      m_new_id = "";
      fault_type = nonfaulty;
   }

void BaseConrtoller::Init(TConfigurationNode& t_node){
   string type = "";
   try{
      TConfigurationNode&  crash_node = GetNode(t_node, "fault");
      GetNodeAttribute(crash_node, "type", type);
      if(type.rfind("crash",0) == 0){
         GetNodeAttribute(crash_node, "m_crash_starttime", m_crash_starttime);
         GetNodeAttribute(crash_node, "m_crash_endtime", m_crash_endtime);
      }
      else if(type == "keep_distance"){
         fault_type = keep_distance;
      }
      else if(type == "virtual_forces_walk_away"){
         fault_type = virtual_forces_walk_away;
      }
      else if(type == "opposite"){
         fault_type = opposite;
      }
   }
   catch(CARGoSException& ex) {
      m_crash_starttime = 100000000;
      m_crash_endtime = 100000000;
   }
   try{
      string type;
      TConfigurationNode&  exploration_node = GetNode(t_node, "exploration");
      GetNodeAttribute(exploration_node, "implementation", type);
      if(type == "random"){
         m_random_exploration = true;
      }
      else{
         m_random_exploration = false;
      }
   }
   catch(CARGoSException& ex) {
      m_random_exploration = false;
   }
    m_crash_time = pcRNG->Uniform(CRange<Real>(m_crash_starttime, m_crash_endtime));
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcLedAct = GetActuator<CCI_EPuck2LEDsActuator>("epuck2_leds");
    m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
    m_pcRABSens = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
    m_pcEncoderSensor = GetSensor  <CCI_EPuck2EncoderSensor    >("epuck2_encoder" );
    try {
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    Reset();
}

string BaseConrtoller::GetId(){
   string input = CCI_Controller::GetId();
   // if(m_new_id == "0"){
      // cout << input << endl;
   // }
   if(m_new_id == ""){
      string numberPart;
      for (int i = input.size() - 1; i >= 0; --i) {
         if (std::isdigit(input[i])) {
            numberPart = input[i] + numberPart;
         } else {
            break;
         }
      }
      m_new_id = to_string(static_cast<uint8_t>(stoi(numberPart)));
   }
   return m_new_id;
}

void BaseConrtoller::Reset() {
   m_eState = STATE_ALONE;
   m_pcRABAct->SetData(0, STATE_ALONE);
   int id = stoi(GetId());
   m_pcRABAct->SetData(1, id);
   m_pcLedAct->SetAllRGBColors(CColor::GREEN);
   m_heading = CVector2::ZERO;
   random_destination = CVector2::ZERO;
}

void BaseConrtoller::Communicate(){
    m_pcRABAct->SetData(0, m_eState);
   UInt8 id = stoi(GetId());
    m_pcRABAct->SetData(1, id);
    if(m_eState == STATE_PAIRED){
      m_heading = CVector2::ZERO;
    }
}

void BaseConrtoller::handleFaultBehaviour(){
   if(ShouldTransitionToPaired()){
        m_eState = STATE_PAIRED;
    }
    else if(ShouldTransitionToAlone()){
        m_eState = STATE_ALONE;
    }
   if(fault_type != nonfaulty){
      m_pcLedAct->SetAllRGBColors(CColor::RED);
      m_pcLedAct->SetAllRedLeds(true);
   }
   if(fault_type == crash){
      m_heading = CVector2::ZERO;
   }
   if(fault_type == virtual_forces_walk_away){
      m_heading = VirtualForceWalkAwayFlockingVector();
   }
   if(fault_type == keep_distance){
      m_heading = KeepDistanceFlockingVector();
   }
   if(fault_type == opposite){
      m_heading = -m_heading;
   }
}

void BaseConrtoller::ControlStep(){
   Communicate();
   handleFaultBehaviour();
   SetWheelSpeedsFromVector(m_heading);
}


void BaseConrtoller::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   Real fHeadingLength = c_heading.Length();
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   //  std::cout << fBaseAngularWheelSpeed << std::endl;

   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */

   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}


void BaseConrtoller::SWheelTurningParams::Init(TConfigurationNode& t_node) {
    try {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}


bool BaseConrtoller::ShouldTransitionToAlone(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    bool has_near_robot = false;
    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Range/100 < PAIRING_THRESHOLD){
                has_near_robot = true;
            }
        }
    }
    return !has_near_robot;
}

bool BaseConrtoller::ShouldTransitionToPaired(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Range/100 < PAIRING_THRESHOLD && tMsgs[i].Data[0] == STATE_ALONE){
                return true;
            }
        }
    }
    return false;
}

CVector2 BaseConrtoller::RandomWalk(){
   if(m_time % 50 == 0 || random_destination == CVector2::ZERO){
      CRadians angle = pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
      Real radius = pcRNG->Uniform(CRange<Real>(0,1));
      random_destination = CVector2(radius * Cos(angle), radius * Sin(angle));
   }
   CVector2 to_point = random_destination - m_position;
   CRadians cZAngle, cYAngle, cXAngle;
   m_orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   to_point.Rotate(-cZAngle);
   return to_point.Normalize();
}


CVector2 BaseConrtoller::KeepDistanceFlockingVector(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(!tMsgs.empty()) {
        CVector2 cAccum;
        Real fLJ;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            double distance = tMsgs[i].Range;
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                fLJ = LennardJonesPotential(tMsgs[i].Range);
                cAccum += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
            else{
                fLJ = distance * (0.0000005 - 40 * ::pow(M_E, -::pow(distance,2)/50)); // Gazi force
                cAccum += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
        }
        return cAccum;
    }
    else {
        return CVector2();
    }
}

CVector2 BaseConrtoller::VirtualForceWalkAwayFlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 cAccum;
        Real fLJ;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            fLJ = -ElectricalForce(tMsgs[i].Range);
            cAccum += CVector2(fLJ, tMsgs[i].HorizontalBearing);
        }
        return cAccum;
    }
    else {
        return CVector2();
    }
}

string toStringFaultType(FaultyType type){
   switch(type) {
      case nonfaulty: return "nonfaulty";
      case crash: return "crash";
      case virtual_forces_walk_away: return "virtual_forces_walk_away";
      case keep_distance: return "keep_distance";
   }
}
