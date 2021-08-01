cmake_minimum_required(VERSION 3.8)
project(nav2_play_plugins)

# Default to c11
set(CMAKE_C_STANDARD 11)

# Default to c++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

# Add include directories
include_directories(
  include
)

# set library name
set(library_name ${PROJECT_NAME})

# set dependencies
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

add_library(${library_name} SHARED
  src/noti_at_waypoint.cpp
  src/straight_line_planner.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Needed for test
add_library(noti_at_waypoint SHARED src/noti_at_waypoint.cpp)
ament_target_dependencies(noti_at_waypoint ${dependencies})
#prevent pluginlib from using boost
target_compile_definitions(noti_at_waypoint PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Needed for test
add_library(straight_line_planner SHARED src/straight_line_planner.cpp)
ament_target_dependencies(straight_line_planner ${dependencies})
#prevent pluginlib from using boost
target_compile_definitions(straight_line_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# NOTE: Must install in target nav2_waypoint_follower
pluginlib_export_plugin_description_file(
  nav2_waypoint_follower
  noti_at_waypoint_plugin.xml
)
# NOTE: Must install in target nav2_core
pluginlib_export_plugin_description_file(
  nav2_core
  global_planner_plugin.xml
)

install(TARGETS ${library_name} noti_at_waypoint straight_line_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  noti_at_waypoint_plugin.xml
  global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
