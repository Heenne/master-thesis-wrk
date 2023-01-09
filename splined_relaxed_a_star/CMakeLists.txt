cmake_minimum_required(VERSION 2.8.3)
project(splined_relaxed_a_star)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  rospy
  std_msgs
  sensor_msgs
  fpp_msgs
  costmap_2d
  nav_core
)

# # System dependencies are found with CMake's conventions
find_package(OpenCV)

find_package(Eigen3 REQUIRED)


# find_package(OpenCV 2 REQUIRED)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES com
  # CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS roscpp rospy std_msgs fpp_msgs nav_core

  # DEPENDS system_lib
)

add_library(splined_relaxed_a_star
  src/splined_relaxed_a_star.cpp
  src/visualization_helper.cpp
  src/ras_param_manager.cpp
  src/quintic_bezier_spline.cpp
  src/cubic_bezier_spline.cpp
  src/base_bezier_spline.cpp
)

target_link_libraries(splined_relaxed_a_star ${catkin_LIBRARIES})

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(

  # include
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
