# Unitree_slam ROS workspace

## Branch decription

In this repository we have three main branches: `main`, `unitree_realsense_slam_release` and `xavier-arm`.

* Branch `main` contains all the work related to the realsense and lidar slam.
* Branch `unitree_realsense_slam_release` is a copy of the `main` branch, but doesn't content the work related to a lidar slam. 
* Branch `xavier-arm` is a slightly changed copy of the `main` branch able to be built on ARM platform (Xavier).
  
## Dependencies

* [ros melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) 
* [unitree_ros](https://github.com/unitreerobotics/unitree_ros) (for sim only)
* [realsense_ros_gazebo](https://github.com/nilseuropa/realsense_ros_gazebo) (for sim only)
* [rtabmap_ros](https://github.com/introlab/rtabmap_ros)
* [realsense2_camera](https://github.com/IntelRealSense/realsense-ros)
* move_base `sudo apt install ros-melodic-move-base`
* [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan)

~/.bashrc :

    source /opt/ros/melodic/setup.bash
    source /usr/share/gazebo-9/setup.sh
    export UNITREE_SDK_VERSION=3_2
    export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk
    export UNITREE_PLATFORM="amd64"

## How to use

### Real robot

    roslaunch realsense_slam unitree_real.launch
### Simulation

    roslaunch realsense_slam unitree_sim.launch
