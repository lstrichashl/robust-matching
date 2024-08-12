link_directories(${CMAKE_BINARY_DIR}/src/controllers/robust_matching)
set(iterated_meeting_points_loop_functions_SOURCES iterated_meeting_points_loop_functions.cpp)

# if(ARGOS_QTOPENGL_FOUND)
#   include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
#   set(iterated_meeting_points_loop_functions_SOURCES
#     ${iterated_meeting_points_loop_functions_SOURCES}
#     matching_qt_user_functions.cpp)
# endif(ARGOS_QTOPENGL_FOUND)

add_library(iterated_meeting_points_loop_functions SHARED ${iterated_meeting_points_loop_functions_SOURCES})
target_link_libraries(iterated_meeting_points_loop_functions
  abstract_controllers
  basic_loop_functions
  robust_matching
  meeting_points
  minimum_cost_perfect_matching
  matching_result
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(iterated_meeting_points_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
