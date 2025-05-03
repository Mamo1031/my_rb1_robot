cmake_minimum_required(VERSION 3.0.2)
project(my_rb1_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES Rotate.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/rotate_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
