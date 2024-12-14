#include "greedy_meeting_points.h"
#include <algorithm>

bool sortBySecond(const pair<int,Real>& a, const pair<int,Real>& b){
    return a.second < b.second;
}

Real GaziRepultion2(double distance){
    if(distance > 20){
        return 0;
    }
    double b = 1;
    double c = 1;
    double v = distance * (- b * ::pow(M_E, -::pow(distance,2)/c));
    return v;
}

Real GaziAttraction2(double distance){
    double a = 0.01;
    return distance * a;
}
Real GaziForce2(double distance){
    return GaziAttraction2(distance) + GaziRepultion2(distance);
}

CGreedyMeetingPoint::CGreedyMeetingPoint():CMeetingPointEpuck(){
    handshake = IDLE;
    other_try_to_sin_robot_id = -1;
    matched_robot_id = "-1";
}


void CGreedyMeetingPoint::ControlStep(){
    string robot_log_id = "272";
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(m_eState == STATE_ALONE){
        if(handshake == IDLE){
            if(! tMsgs.empty()) {
                vector<pair<int,Real>> neighboring_idle_robots;
                vector<pair<int,Real>> neighboring_sin_me_robots;
                for(size_t i = 0; i < tMsgs.size(); ++i) {
                    if(GetId() == robot_log_id){
                        cout << "message: "  << tMsgs[i].Data[0] << " " << tMsgs[i].Data[1] << " "<< tMsgs[i].Data[2] << " "<< tMsgs[i].Data[3] << endl;
                    }
                    if(tMsgs[i].Data[2] == SIN && tMsgs[i].Data[3] == stoi(GetId())){
                        neighboring_sin_me_robots.push_back(make_pair(tMsgs[i].Data[1], tMsgs[i].Range));
                    }
                    if(tMsgs[i].Data[0] == STATE_ALONE && tMsgs[i].Data[2] == IDLE) {
                        neighboring_idle_robots.push_back(make_pair(tMsgs[i].Data[1], tMsgs[i].Range));
                    }
                }
                if(neighboring_sin_me_robots.size() > 0){
                    std::sort(neighboring_sin_me_robots.begin(), neighboring_sin_me_robots.end(), sortBySecond);
                    for(int j = 0; j < neighboring_sin_me_robots.size(); j++){
                        if(m_matched_robot_indexes.find(neighboring_sin_me_robots[j].first) == m_matched_robot_indexes.end()){
                            handshake = SINACK;
                            other_try_to_sin_robot_id = neighboring_sin_me_robots[j].first;
                            if(GetId() == robot_log_id){
                                cout << "bbb" << other_try_to_sin_robot_id << endl;
                            }
                            break;
                        }
                    }
                }
                else if(neighboring_idle_robots.size() > 0){
                    std::sort(neighboring_idle_robots.begin(), neighboring_idle_robots.end(), sortBySecond);
                    for(int j = 0; j < neighboring_idle_robots.size(); j++){
                        if(m_matched_robot_indexes.find(neighboring_idle_robots[j].first) == m_matched_robot_indexes.end()){
                            handshake = SIN;
                            other_try_to_sin_robot_id = neighboring_idle_robots[j].first;
                            if(GetId() == robot_log_id){
                                cout << "aaa" << other_try_to_sin_robot_id << endl;
                            }
                            break;
                        }
                    }
                }
            }
        }
        else if(handshake == SIN){
            const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
            bool got_sinack = false;
            if(! tMsgs.empty()) {
                for(size_t i = 0; i < tMsgs.size() && !got_sinack; ++i) {
                    if(tMsgs[i].Data[1] == other_try_to_sin_robot_id && 
                        (tMsgs[i].Data[2] == SINACK || tMsgs[i].Data[2] == SIN) && 
                        tMsgs[i].Data[3] == stoi(GetId())){
                        got_sinack = true;
                    }
                }
            }
            if(got_sinack){
                handshake = ACK;
                matched_robot_id = to_string(other_try_to_sin_robot_id);
            }
            else{
                handshake = IDLE;
                other_try_to_sin_robot_id = -1;
            }
        }
        else if(handshake == SINACK){
            const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
            bool got_ack = false;
            if(! tMsgs.empty()) {
                for(size_t i = 0; i < tMsgs.size() && !got_ack; ++i) {
                    if(tMsgs[i].Data[1] ==  other_try_to_sin_robot_id &&
                        (tMsgs[i].Data[2] == ACK || tMsgs[i].Data[2] == SINACK) &&
                        tMsgs[i].Data[3] == stoi(GetId())){
                        got_ack = true;
                    }
                }
            }
            if(got_ack){
                handshake = ACK;
                matched_robot_id = to_string(other_try_to_sin_robot_id);
            }
            else{
                handshake = IDLE;
                other_try_to_sin_robot_id = -1;
            }
        }

        Real distance_to_meeting_point = (m_meeting_point - m_position).Length();
        if(distance_to_meeting_point < 0.035 && handshake == ACK){
            handshake = WAIT_FOR_PARTER_IN_TARGET;
            m_heading = CVector2::ZERO;
            time_for_wait_for_parter_in_target = 0;
        }
        if(handshake == WAIT_FOR_PARTER_IN_TARGET) {
            if(distance_to_meeting_point > 0.035){
                handshake = ACK;
                time_for_wait_for_parter_in_target = 0;
            }
            m_heading = CVector2::ZERO;
            time_for_wait_for_parter_in_target += 1;
            if(time_for_wait_for_parter_in_target >= 200) {
                m_meeting_point = CVector2::ZERO;
                m_matched_robot_indexes.insert(stoi(matched_robot_id));
                handshake = IDLE;
                other_try_to_sin_robot_id = -1;
                matched_robot_id = "-1";
                time_for_wait_for_parter_in_target = 0;
            }
        }
        headingUpdate();
    }
    if(GetId() == robot_log_id){
        // cout << GetId() << " matched:" << matched_robot_id << " TCP:" << handshake << " try:" << other_try_to_sin_robot_id << " e_state:" << m_eState << endl;
        // CVector2 distance_to_meeting_point = m_meeting_point - m_position;
        // cout << m_heading.GetX() << ", " << m_heading.GetY() << " " <<  distance_to_meeting_point.Length() << endl;
        // Real heading_d = m_heading.Length() - m_prev_heading.Length();
        // cout << heading_d.GetX() << ", " << heading_d.GetY() << " " <<  heading_d.Length() << endl;
        // cout << heading_d << endl;
    }
    m_pcRABAct->SetData(2, handshake);
    m_pcRABAct->SetData(3, other_try_to_sin_robot_id);
    BaseConrtoller::ControlStep();
}

void CGreedyMeetingPoint::headingUpdate(){
    CVector2 random = RandomWalk();
    Real heading_d = m_heading.Length() - m_prev_heading.Length();
    m_prev_heading = m_heading;

    m_heading = 2 * m_heading + 10 * heading_d * m_heading;
    if(handshake != WAIT_FOR_PARTER_IN_TARGET){
        CVector2 flocking = FlockingVector() * 10;
        m_heading += flocking;
        if(handshake != ACK){
            m_heading = random + flocking;
            m_heading *= 5;
        }
        else{
            m_heading += random / 5;
        }
    }
}

CVector2 CGreedyMeetingPoint::FlockingVector() {
    return CMeetingPointEpuck::FlockingVector();
    // if(matched_robot_id != "-1"){
    //     return CMeetingPointEpuck::FlockingVector();
    // }
    // else{
    //     CVector2 gaziForce;
    //     // const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    //     // for(size_t i = 0; i < tMsgs.size(); ++i) {
    //     //     Real force = GaziForce2(tMsgs[i].Range);
    //     //     gaziForce += CVector2(force, tMsgs[i].HorizontalBearing);
    //     // }
    //     return gaziForce.Normalize();
    // }
}



REGISTER_CONTROLLER(CGreedyMeetingPoint, "greedy_meeting_point_controller")
