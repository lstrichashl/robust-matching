#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <iostream>
 
using namespace std;

enum EState {
   STATE_ALONE = 0,
   STATE_PAIRED
};

class BaseConrtoller{

public:
   virtual std::string GetType(){
      return m_typename;
   }
   virtual EState GetEState(){
      return m_eState;
   }
   double PAIRING_THRESHOLD = 0.07;
   
protected:
    std::string m_typename;
    EState m_eState;

};

#endif