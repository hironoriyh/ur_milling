cmake_minimum_required(VERSION 2.8.3)
project(ur3_milling)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geometric_shapes
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  std_msgs
  sensor_msgs
  tf
  tf_conversions
  object_detection
  visualization_msgs
  urdf_models
  grasp_and_place_msgs
  cv_bridge
  ur_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
#find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    geometric_shapes
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
    std_msgs
    sensor_msgs
    tf
    tf_conversions
    object_detection
    visualization_msgs
    urdf_models
    grasp_and_place_msgs
    cv_bridge
    ur_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(
#   ${PROJECT_NAME}
#   src/GripperInterface.cpp
#   src/GraspObject.cpp
#   src/Object.cpp
#   src/PlaceObject.cpp
#   # src/regrip.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(
# grasp_and_place
# object_detection_generate_messages_cpp
# )

## Declare a C++ executable
add_executable(
  milling_path
  src/milling_path.cpp
  # src/milling_path_node.cpp
)

add_executable(
  milling_path_node
  src/MillingPath.cpp
  src/milling_path_node.cpp
)

# add_executable(
#   set_grasping_pose_node
#   src/SetGraspingPose.cpp
#   src/set_grasping_pose_node.cpp
# )

# add_executable(
#   camera_calibration_node
#   src/CameraCalibration.cpp
#   src/camera_calibration_node.cpp
# )

# add_executable(
#   object_tf_publisher
#   src/ObjectTFPublisher.cpp
# )


## Specify libraries to link a library or executable target against
target_link_libraries(
  milling_path
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(
  milling_path_node
  # ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
#
# target_link_libraries(
#   stack_each_node
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
#
# target_link_libraries(
#   set_grasping_pose_node
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   camera_calibration_node
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   object_tf_publisher
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
