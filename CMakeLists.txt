cmake_minimum_required(VERSION 2.8.3)
project(rohand)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS common
  LIBRARIES rohand
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/common/)