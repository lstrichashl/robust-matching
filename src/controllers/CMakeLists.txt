add_subdirectory(abstract_controllers)

add_subdirectory(robust_matching)
# add_subdirectory(move_to_point)
add_subdirectory(virtual_forces)

# set(controllers_SOURCES adverserial/opposite_direction/opposite_direction.h adverserial/opposite_direction/opposite_direction.cpp)
# set(controllers_SOURCES
#     ${controllers_SOURCES}
#     robust_matching/matching_regular_bot.h robust_matching/matching_regular_bot.cpp)
# set(controllers_SOURCES
#     ${controllers_SOURCES}
#     virtual_forces/adverserial/adverserial.h virtual_forces/adverserial/adverserial.cpp)
# set(controllers_SOURCES
#     ${controllers_SOURCES}
#     virtual_forces/virtual_forces_bot/regular_bot.h virtual_forces/virtual_forces_bot/regular_bot.cpp)
# add_library(robot_controllers SHARED ${controllers_SOURCES})

# target_link_libraries(robot_controllers
#   argos3core_simulator
#   argos3plugin_simulator_epuck2
#   argos3plugin_simulator_genericrobot)
