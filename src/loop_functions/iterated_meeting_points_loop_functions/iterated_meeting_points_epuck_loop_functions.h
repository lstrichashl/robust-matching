#ifndef ITERATED_MEETING_POINTS_LOOP_FUNCTIONS_H
#define ITERATED_MEETING_POINTS_LOOP_FUNCTIONS_H

#include "src/loop_functions/print_experiment_loop_fuctions/print_experiment_loop_fuctions.h"
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/meeting_points/meeting_points_epuck.h"

using namespace argos;

class CIteratedMeetingPointsEpuck : public CPrintExperimentFunctions {

    public:
        CIteratedMeetingPointsEpuck();
        virtual ~CIteratedMeetingPointsEpuck() {}
        virtual void PreStep();
        virtual void Reset();
        virtual void UpdateMatching();

    private:
        Graph m_robotGraph;
        vector<int> m_matching;
        vector<double> m_costs;
        vector<CEntity*> m_robots_in_matching;
        double m_matching_max_cost;
        int m_start_time_iterval;
};


#endif