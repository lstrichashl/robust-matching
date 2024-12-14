add_library(triplet SHARED forces/triplet_forces.h 
                            forces/triplet_forces.cpp 
            )
target_link_libraries(triplet
  abstract_controllers
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)