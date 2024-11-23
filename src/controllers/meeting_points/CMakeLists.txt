add_library(meeting_points SHARED meeting_points.h 
                                  meeting_points.cpp
                                  meeting_points_epuck.h
                                  meeting_points_epuck.cpp
                                  greedy_meeting_points.h
                                  greedy_meeintg_points.cpp)
target_link_libraries(meeting_points
  abstract_controllers
  argos3core_simulator
  argos3plugin_simulator_genericrobot)