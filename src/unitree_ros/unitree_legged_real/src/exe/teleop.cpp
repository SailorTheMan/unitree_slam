/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include <geometry_msgs/Twist.h>

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif



const int running_time = 30000;


geometry_msgs::Twist teleop_cmd;

void cmd_velCallback(const geometry_msgs::Twist msg)
{
//   ROS_INFO("I heard: [%f]", msg.linear.x);
    teleop_cmd = msg;
}



template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);

    while (ros::ok()){
        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        // printf("%f\n",  RecvHighROS.forwardSpeed);

        SendHighROS.forwardSpeed = 0.0f;
        SendHighROS.sideSpeed = 0.0f;
        SendHighROS.rotateSpeed = 0.0f;
        SendHighROS.bodyHeight = 0.0f;

        SendHighROS.mode = 0; // 0 , 1 , 2
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        if (motiontime<1000) {
            ROS_INFO("mode 0");
        }
        if (motiontime>=1000 && motiontime<2000){
            SendHighROS.mode = 2;
            ROS_INFO("mode 1");
        }
        if (motiontime>=2000 && motiontime<2200){
            SendHighROS.mode = 2;
            ROS_INFO("mode 2");
        }
        if (motiontime>=2200 && motiontime<running_time){
            SendHighROS.mode = 2;
            if (teleop_cmd.linear.x == 0.0f){
                SendHighROS.forwardSpeed = teleop_cmd.linear.x;
            }
            else {
                SendHighROS.forwardSpeed = teleop_cmd.linear.x / 5.0f;
            }
            if (teleop_cmd.linear.y == 0.0f){
                SendHighROS.sideSpeed = teleop_cmd.linear.y;
            }
            else {
                SendHighROS.sideSpeed = teleop_cmd.linear.y / 5.0f;
            }
            if (teleop_cmd.angular.z == 0.0f){
                SendHighROS.rotateSpeed = teleop_cmd.angular.z;
            }
            else {
                SendHighROS.rotateSpeed = teleop_cmd.angular.z / 10.0f;
            }           
            ROS_INFO("mode 2   forward: [%f], side: [%f], rotate: [%f]", SendHighROS.forwardSpeed, SendHighROS.sideSpeed, SendHighROS.rotateSpeed);
        }
        if (motiontime>=running_time && motiontime<running_time+1000) {
            SendHighROS.mode = 1;
            ROS_INFO("mode 1");
        }
        if (motiontime>=running_time+1000) {
            SendHighROS.mode = 0;
            ROS_INFO("mode 0");
        }

        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    #ifdef SDK3_1
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
    
}