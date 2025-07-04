cmake_minimum_required(VERSION 3.8)
project(ur10e_lift_trajectory_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# Original IK Trajectory Planner
add_executable(ik_trajectory_planner src/ik_trajectory_planner.cpp)
ament_target_dependencies(ik_trajectory_planner 
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs)

# New Spline Trajectory Planner
add_executable(spline_trajectory_planner src/spline_trajectory_planner.cpp)
ament_target_dependencies(spline_trajectory_planner 
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs)

install(TARGETS
  ik_trajectory_planner
  spline_trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/waypoints")
  install(DIRECTORY waypoints DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()