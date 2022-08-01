cmake_minimum_required(VERSION 3.0.2)
project(eskf)

set(geography_lib lib/common.cpp lib/geography.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  lib
  include
  ${PROJECT_SOURCE_DIR}/../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(geography ${geography_lib})

add_executable(${PROJECT_NAME} src/run.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  geography
)