add_library(virtual_forces SHARED forces_moving.h 
                                  forces_moving.cpp )
target_link_libraries(forces_moving
  abstract_controllers
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)