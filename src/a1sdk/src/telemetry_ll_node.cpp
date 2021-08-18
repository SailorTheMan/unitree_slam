/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/


    // TODO: 
    // Need to visualize:
    // > Each leg state

    // FIXME:
    // > 
    


#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PolygonStamped.h"       // represents leg positions
#include "geometry_msgs/PointStamped.h"

using namespace UNITREE_LEGGED_SDK;

// FR FL RR RL
std::string footFrames[4] = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};

class ROS_Publishers
{
public:
    ros::Publisher *chatter;
    ros::Publisher *joint_states_pub;
    ros::Publisher *leg_force_pub[4];
    int seq;
};

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl(ROS_Publishers rospub);

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl(ROS_Publishers rospub) 
{
    if (!ros::ok()) exit(1) ;       // probably forbidden technique, but it works 
    sensor_msgs::JointState joint_state_msg;
    geometry_msgs::WrenchStamped legForces[4];  // foot forces

    motiontime += 1;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
    /*      Body        */

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    for (int leg = 0; leg < 4; leg++)
    {
        // Constructing leg forces messages
        legForces[leg].wrench.force.z = state.footForce[leg];       // may be either y or z  // footForceEst ??
        legForces[leg].header.frame_id = footFrames[leg];
        legForces[leg].header.seq = rospub.seq;
        legForces[leg].header.stamp.now(); //= ros::Time::now();
        rospub.leg_force_pub[leg]->publish(legForces[leg]);
    }

    joint_state_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    std::vector<double> joint_pos_vec;
    std::vector<double> joint_vel_vec;
    std::vector<double> joint_tor_vec;

    for (int motor = 0; motor < 12; motor++)                        // 3 mototrs x 4 legs
    {
        joint_pos_vec.push_back(state.motorState[motor].q);         // cur angle
        joint_vel_vec.push_back(state.motorState[motor].dq);        // cur velocity
        joint_tor_vec.push_back(state.motorState[motor].tauEst);    // cur torgue
    } 
    
    joint_state_msg.position = joint_pos_vec;
    joint_state_msg.velocity = joint_vel_vec;
    joint_state_msg.effort   = joint_tor_vec;

    
    joint_state_msg.header.frame_id = ' ';
    joint_state_msg.header.seq = rospub.seq;
    joint_state_msg.header.stamp = ros::Time::now();

    rospub.joint_states_pub->publish(joint_state_msg);
    

    /*        End Body      */
    //if (ros::ok()) SendToROS(this, rospub);

    if (motiontime > 10)
    {
        safe.PositionLimit(cmd);
        safe.PowerProtect(cmd, state, 1);
        // You can uncomment it for position protection
        // safe.PositionProtect(cmd, state, 0.087);
    }   
    rospub.seq++;
    ros::spinOnce();
    udp.SetSend(cmd);
}

int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "a1Telemetry");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub;
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher FRf_pub = nh.advertise<geometry_msgs::WrenchStamped>("FR_force", 1000);
    ros::Publisher FLf_pub = nh.advertise<geometry_msgs::WrenchStamped>("FL_force", 1000);
    ros::Publisher RRf_pub = nh.advertise<geometry_msgs::WrenchStamped>("RR_force", 1000);
    ros::Publisher RLf_pub = nh.advertise<geometry_msgs::WrenchStamped>("RL_force", 1000);
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    

    /* ROS structure construction for loop */
    ROS_Publishers rospub;      // a structure to pass into loop control
    rospub.chatter = &chatter_pub;
    rospub.leg_force_pub[0] = &FRf_pub;    // packing all neccessary ros objects together
    rospub.leg_force_pub[1] = &FLf_pub;
    rospub.leg_force_pub[2] = &RRf_pub;
    rospub.leg_force_pub[3] = &RLf_pub;
    rospub.joint_states_pub = &joint_states_pub;    // packing all neccessary ros objects together
    rospub.seq = 0;

    //Custom custom(HIGHLEVEL);
    Custom custom(LOWLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom, rospub));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));
    
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(1);
    };

    return 0; 
}
