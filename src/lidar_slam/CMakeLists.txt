cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
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
  CATKIN_DEPENDS roscpp sensor_msgs  
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(flat_imu_node src/flat_imu_node.cpp)
add_dependencies(flat_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flat_imu_node ${catkin_LIBRARIES})

#############
## Install ##
#############
