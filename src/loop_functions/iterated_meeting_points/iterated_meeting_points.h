#ifndef ITERATED_MEETING_POINTS_LOOP_FUNCTIONS_H
#define ITERATED_MEETING_POINTS_LOOP_FUNCTIONS_H

#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"


using namespace argos;

class CIteratedMeetingPoints : public CBasicLoopFunctions {

    public:
        CIteratedMeetingPoints();
        virtual ~CIteratedMeetingPoints() {}
        virtual void PreStep();


};


#endif