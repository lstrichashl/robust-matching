#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <iostream>
 
using namespace std;

class BaseConrtoller{

public:
   virtual std::string GetType(){
      return m_typename;
   }


protected:
    std::string m_typename;

};

#endif