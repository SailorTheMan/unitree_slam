cmake_minimum_required(VERSION 3.0.2)
project(realsense_slam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs 
  tf 
  tf2_ros
)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs tf tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_handler_node src/odom_handler.cpp)
add_dependencies(odom_handler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_handler_node ${catkin_LIBRARIES})

#############
## Install ##
#############
