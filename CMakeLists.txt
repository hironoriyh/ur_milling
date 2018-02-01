cmake_minimum_required(VERSION 2.8.3)
project(ur3_milling)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  std_msgs
  # sensor_msgs
  # tf
  # tf_conversions
  # object_detection
  # visualization_msgs
  # urdf_models
  # cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometric_shapes
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
    std_msgs
    # sensor_msgs
    # tf
    # tf_conversions
    # object_detection
    # visualization_msgs
    # cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
)


add_executable(miling_path_test src/milling_path.cpp)
target_link_libraries(miling_path_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install(TARGETS ur3_milling DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


## Declare a C++ executable
# add_executable(
#   serial_stacking_node
#   src/SerialStacking.cpp
#   src/serial_stacking_node.cpp
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
#
# target_link_libraries(
#   serial_stacking_node
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#)
