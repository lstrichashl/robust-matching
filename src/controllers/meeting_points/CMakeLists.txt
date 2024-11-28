add_library(meeting_points SHARED meeting_points.h 
                                  meeting_points.cpp
                                  meeting_points_epuck.h
                                  meeting_points_epuck.cpp
                                  distributed/greedy_meeting_points.h
                                  distributed/greedy_meeintg_points.cpp
                                  distributed/adverserial/walk_away.h
                                  distributed/adverserial/walk_away.cpp
                                  distributed/nieberg.cpp
                                  distributed/nieberg.h)
target_link_libraries(meeting_points
  abstract_controllers
  argos3core_simulator
  argos3plugin_simulator_genericrobot)