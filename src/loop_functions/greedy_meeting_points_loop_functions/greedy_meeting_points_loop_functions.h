#ifndef GREEDY_MEETING_POINTS_LOOP_FUNCTIONS_H
#define GREEDY_MEETING_POINTS_LOOP_FUNCTIONS_H

#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/meeting_points/greedy_meeting_points.h"
#include <unordered_map>

using namespace argos;

class CGreedyMeetingPointsLoopFunctions : public CBasicLoopFunctions {

    public:
        CGreedyMeetingPointsLoopFunctions();
        virtual ~CGreedyMeetingPointsLoopFunctions() {}
        virtual void PostStep();
        virtual void Reset();
        virtual void Init(TConfigurationNode& t_tree);

    protected:
        std::unordered_map<string, int> m_robot_id_to_index;
};


#endif