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
include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/demo_ZedCamera_node.cpp)
add_executable(zed_cornhole src/zed_cornhole.cpp)
# add_executable(blob_detection src/blob_detection.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(zed_cornhole 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    )


