add_library(virtual_forces SHARED virtual_forces_bot.h 
                                  virtual_forces_bot.cpp 
                                  virtual_forces_adverserial.h 
                                  virtual_forces_adverserial.cpp
                                  keep_distance.h
                                  keep_distance.cpp
                                  virtual_forces_random.h
                                  virtual_forces_random.cpp
                                  tit_for_tat.h
                                  tit_for_tat.cpp)
target_link_libraries(virtual_forces
  abstract_controllers
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)