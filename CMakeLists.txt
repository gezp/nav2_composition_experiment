cmake_minimum_required(VERSION 3.5)
project(nav2_composition_experiment)

find_package(ament_cmake REQUIRED)
find_package(navigation2 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

add_executable(composed_bringup
  src/composed_bringup.cpp
)

set(dependencies
  nav2_map_server
  nav2_amcl
  nav2_controller
  nav2_smoother
  nav2_planner
  nav2_recoveries
  nav2_bt_navigator
  nav2_waypoint_follower
  nav2_lifecycle_manager
)

ament_target_dependencies(composed_bringup
  ${dependencies}
)

add_executable(
  component_container_isolated2
  src/component_container_isolated2.cpp
)
ament_target_dependencies(component_container_isolated2
  rclcpp_components
)

install(TARGETS composed_bringup component_container_isolated2
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
