cmake_minimum_required(VERSION 2.8.12)
project(unity_pr2_beginner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  bio_ik
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  message_filters
  std_msgs
  geometry_msgs
  sensor_msgs
  pr2_controllers_msgs
  actionlib
  rosbag
)
find_package(Eigen3 REQUIRED)


catkin_package(CATKIN_DEPENDS  std_msgs)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(pr2_right_arm_control src/pr2_right_arm_control.cpp)
add_dependencies(pr2_right_arm_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr2_right_arm_control ${catkin_LIBRARIES} ccd)
