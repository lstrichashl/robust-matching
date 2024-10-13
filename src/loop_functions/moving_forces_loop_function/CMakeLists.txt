link_directories(${CMAKE_BINARY_DIR}/src/controllers/robust_matching)
set(moving_forces_loop_function_SOURCES moving_forces_loop_function.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(moving_forces_loop_function_SOURCES
    ${moving_forces_loop_function_SOURCES}
    matching_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(moving_forces_loop_function SHARED ${moving_forces_loop_function_SOURCES})
target_link_libraries(moving_forces_loop_function
  abstract_controllers
  basic_loop_functions
  robust_matching
  minimum_cost_perfect_matching
  matching_result
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(moving_forces_loop_function argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
