link_directories(${CMAKE_BINARY_DIR}/src/controllers/robust_matching)
set(basic_loop_functions_SOURCES basic_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(basic_loop_functions_SOURCES
    ${basic_loop_functions_SOURCES}
    basic_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(basic_loop_functions SHARED ${basic_loop_functions_SOURCES})
target_link_libraries(basic_loop_functions
  virtual_forces
  minimum_cost_perfect_matching
  matching_result
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(basic_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
