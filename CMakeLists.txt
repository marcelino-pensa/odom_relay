cmake_minimum_required(VERSION 2.8.3)
project(odom_relay)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  cmake_modules
  std_msgs
  tf
)

find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odom_relay
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf 
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(odom_relay_lpe src/odom_relay_lpe.cpp)
add_dependencies(odom_relay_lpe ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_relay_lpe ${catkin_LIBRARIES})

add_executable(odom_relay_tf src/odom_relay_tf.cpp)
add_dependencies(odom_relay_tf ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_relay_tf ${catkin_LIBRARIES})

add_executable(odom_relay_twist_body src/odom_relay_twist_body.cpp)
add_dependencies(odom_relay_twist_body ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_relay_twist_body ${catkin_LIBRARIES})

add_executable(odom_relay_pose src/odom_relay_pose.cpp)
add_dependencies(odom_relay_pose ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_relay_pose ${catkin_LIBRARIES})

install(TARGETS odom_relay_twist_body
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})