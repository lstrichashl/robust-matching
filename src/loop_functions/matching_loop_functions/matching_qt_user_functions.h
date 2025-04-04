#ifndef MATCHING_QT_USER_FUNCTIONS_H
#define MATCHING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include "matching_loop_functions.h"

using namespace argos;

class CMatchingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CMatchingQTUserFunctions();

   virtual ~CMatchingQTUserFunctions() {}

   virtual void DrawInWorld();

private:
   CMatchingLoopFunctions& m_matchingLoopFunctions;

};

#endif
