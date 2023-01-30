cmake_minimum_required(VERSION 3.0.2)
project(prm_global_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
  splined_voronoi
)

# # System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

set(tinysplinecxx_DIR "/usr/local/lib64/cmake/tinysplinecxx/")
find_package(tinysplinecxx)

# set(nlopt_DIR "/usr/local/lib/cmake/nlopt/")
find_package(NLopt)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nav_core roscpp rospy std_msgs splined_voronoi
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TINYSPLINECXX_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)




## Declare a C++ library
add_library(${PROJECT_NAME}
   src/PRM_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TINYSPLINECXX_LIBRARIES}
  ${NLOPT_LIBRARIES}
)
