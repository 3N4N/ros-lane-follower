cmake_minimum_required(VERSION 3.0.2)
project(auto_drive VERSION 0.0.1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  genmsg
  cv_bridge
  image_transport
)
find_package(OpenCV 4.2.0 REQUIRED)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

## Declare a catkin package
catkin_package(INCLUDE_DIRS inc
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS OpenCV
)

## Build talker and listener
include_directories(
  include ${catkin_INCLUDE_DIRS}
  /usr/include/opencv4/
)

add_executable(automate_vehicle
  src/automate_vehicle.cc
)
target_link_libraries(automate_vehicle ${catkin_LIBRARIES})
add_dependencies(automate_vehicle
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
