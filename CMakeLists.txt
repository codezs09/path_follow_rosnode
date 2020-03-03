cmake_minimum_required(VERSION 2.8.3)
project(path_follow)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        sensor_msgs
	geometry_msgs
	styx_msgs
	tf
        tf2
	tf2_ros
	tf2_geometry_msgs
	autoware_msgs
	autoware_config_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  libwaypoint_follower
  CATKIN_DEPENDS
  DEPENDS PCL
  ${PCL_OPENMP_PACKAGES}
  roscpp
  message_runtime
  pcl_ros
  std_msgs
  styx_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  autoware_msgs
  autowre_config_msgs
)

#catkin_python_setup()
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower lib/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower
styx_msgs_generate_messages_cpp) 
add_dependencies(libwaypoint_follower
autoware_msgs_generate_messages_cpp)

add_executable(path_follow src/PIDFollow.cpp src/PIDFollow_node.cpp)

target_link_libraries(path_follow  libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(path_follow
styx_msgs_generate_messages_cpp)
add_dependencies(path_follow
autoware_msgs_generate_messages_cpp)
set_target_properties(path_follow PROPERTIES OUTPUT_NAME path_follow PREFIX "")
target_link_libraries(libwaypoint_follower ${catkin_LIBRARIES})

add_executable(odom_relay  src/odom_relay.cpp)
target_link_libraries(odom_relay ${catkin_LIBRARIES})


