link_directories(${CMAKE_BINARY_DIR}/controllers/eyebot_matching)
set(matching_loop_functions_SOURCES matching_loop_functions.cpp)
set(matching_loop_functions_SOURCES
    ${matching_loop_functions_SOURCES}
    BinaryHeap.cpp)
set(matching_loop_functions_SOURCES
    ${matching_loop_functions_SOURCES}
    Graph.cpp)
set(matching_loop_functions_SOURCES
    ${matching_loop_functions_SOURCES}
    Matching.cpp)

# if(ARGOS_QTOPENGL_FOUND)
#   include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
#   set(matching_loop_functions_SOURCES
#     ${matching_loop_functions_SOURCES}
#     matching_qt_user_functions.cpp)
# endif(ARGOS_QTOPENGL_FOUND)

add_library(matching_loop_functions MODULE ${matching_loop_functions_SOURCES})
target_link_libraries(matching_loop_functions
  robust_matching
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

# if(ARGOS_QTOPENGL_FOUND)
#   target_link_libraries(matching_loop_functions argos3plugin_simulator_qtopengl)
# endif(ARGOS_QTOPENGL_FOUND)

# add_subdirectory(minimum_cost_perfect_matching)

