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

    `roslaunch realsense_slam unitree_real.launch`

### Simulation

    `roslaunch realsense_slam unitree_sim.launch`
