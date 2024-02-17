add_library(move_to_point SHARED move_to_point.h move_to_point.cpp)
target_link_libraries(move_to_point
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)
