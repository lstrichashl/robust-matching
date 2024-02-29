#ifndef BASIC_QT_USER_FUNCTIONS_H
#define BASIC_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"

using namespace argos;

class CBasicQTUserFunctions : public CQTOpenGLUserFunctions {

public:
   CBasicQTUserFunctions();
   virtual ~CBasicQTUserFunctions() {}
   virtual void DrawInWorld();

private:
   CBasicLoopFunctions& m_loop_functions;

};

#endif
