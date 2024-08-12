#ifndef ITERATED_MEETING_POINTS_LOOP_FUNCTIONS_H
#define ITERATED_MEETING_POINTS_LOOP_FUNCTIONS_H

#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"
#include <argos3/plugins/robots/eye-bot/simulator/eyebot_entity.h>
#include "src/controllers/meeting_points/meeting_points.h"

using namespace argos;

class CIteratedMeetingPoints : public CBasicLoopFunctions {

    public:
        CIteratedMeetingPoints();
        virtual ~CIteratedMeetingPoints() {}
        virtual void PreStep();
        virtual void Reset();
        virtual void UpdateMatching();

    private:
        Graph m_robotGraph;
        vector<int> m_matching;
        vector<double> m_costs;
        vector<CEntity*> m_robots_in_matching;
};


#endif