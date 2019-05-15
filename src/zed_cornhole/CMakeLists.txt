cmake_minimum_required(VERSION 2.8.3)
project(zed_cornhole)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  pcl_conversions
  pcl_ros
)


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)


## catkin specific configuration 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_ZedCamera
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Build

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

# link_directories(${PCL_LIBRARY_DIRS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo_ZedCamera.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/demo_ZedCamera_node.cpp)
add_executable(zed_cornhole src/zed_cornhole.cpp)
# add_executable(blob_detection src/blob_detection.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(zed_cornhole ${catkin_LIBRARIES} ${OpenCV_LIBS})


