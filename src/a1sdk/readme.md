# Overview:

This package consists of several ROS nodes designed to work with Unitree A1 quadruped robot through its original SDK. Nodes use LCM to communicate with the robot, so they can be run on any machine in it's wifi network. 

`telemetry_hl_node` - Runs SDK 'walk' example or nothing (useful to collect telemetry while operating with wireless controller) depending on the argument, collects and publishes robot high-level telemetry to corresponding topics. The telemetry includes:
- IMU data
- Each foot sensor force value
- Each leg position and velocity relative to body
- Raw odometry 

`telemetry_ll_node` - Publishes low-level telemetry like each motor position and torgue. Useless at the moment since with no LL input the robot would just collapse

`telemetry_teleop_node` - Publishes HL telemetry and receives /cmd_vel messages allowing to teleoperate the robot.

`draw_steps` - A tool that takes foot sensors and leg positions polygon from telemetry_hl and publishes ground contact points for each leg. 

`point_to_odom` - A tool that takes raw odometry  and IMU from telemetry_hl and publishes Odometry_msg. Pretty reliable.

`quatToEuler` - A simple tool to republish IMU orientation quaternion in euler angles for debugging.

Note: to see some of the telemetry  data in rviz the A1 xacro description along with TF data are required. You can launch joint_ & robot_ state_publisher's for manual joint positions as in `a1_telemetry.launch`

# Requirements:

unitree_legged_sdk

    


    
