#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string>
#include <stdio.h>  
#include <tf/transform_datatypes.h>
// #include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Odometry.h"

geometry_msgs::Twist teleop_cmd;

void cmd_velCallback(const geometry_msgs::Twist msg)
{
    // ROS_INFO("I heard: [%f]", msg.linear.x);
    teleop_cmd = msg;
}

int main(int argc, char **argv)
{
    enum coord
    {
        WORLD,
        ROBOT
    };
    coord def_frame = coord::WORLD;

    ros::init(argc, argv, "move_publisher");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, cmd_velCallback);
    

    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

    gazebo_msgs::ModelState model_state_pub;
    

    std::string robot_name;
    ros::param::get("/robot_name", robot_name);
    std::cout << "robot_name: " << robot_name << std::endl;
    
    
    model_state_pub.model_name = robot_name + "_gazebo";
    ros::Rate loop_rate(1000);

    if(def_frame == coord::WORLD)
    {
        model_state_pub.pose.position.x = 1.0;
        model_state_pub.pose.position.y = 0.0;
        model_state_pub.pose.position.z = 0.3;
        
        model_state_pub.pose.orientation.x = 0.0;
        model_state_pub.pose.orientation.y = 0.0;
        model_state_pub.pose.orientation.z = 0.0;
        model_state_pub.pose.orientation.w = 1.0;

        model_state_pub.reference_frame = "world";
          
        long long time_ms = 0;  //time, ms
        const double period = 5000; //ms
        const double radius = 1.5;    //m
        tf::Quaternion q;
        float i = 1.0;
        


        while(ros::ok())
        {
            
            if (teleop_cmd.linear.x == 0.0f){
                teleop_cmd.linear.x = 0.00001f;
            }
            if (teleop_cmd.linear.y == 0.0f){
                teleop_cmd.linear.y = 0.00001f;
            }
            if (teleop_cmd.angular.z == 0.0f){
                teleop_cmd.angular.z = 0.000001f;
            }
            
            
            
            double roll, pitch, yaw;
            tf::Quaternion RQ2;  
            tf::quaternionMsgToTF(model_state_pub.pose.orientation,RQ2);
            tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  
            
            // ROS_INFO("teleop_cmd.linear.y: [%f]", teleop_cmd.linear.y);
            


            model_state_pub.pose.position.x += (teleop_cmd.linear.x / 500.0f) * cos((yaw)) - 
                                                (teleop_cmd.linear.y / 500.0f) * sin((yaw));
            model_state_pub.pose.position.y += teleop_cmd.linear.x / 500.0f * sin((yaw)) + 
                                                (teleop_cmd.linear.y / 500.0f) * cos((yaw));
            double new_yaw;
            new_yaw = yaw + teleop_cmd.angular.z / 1000.0;
            if (yaw > 3.1415){
                yaw = -3.1415;
            }
            if (yaw < -3.1415){
                yaw = 3.1415;
            }



            geometry_msgs::Quaternion quat;
            quat=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,new_yaw);
            
            model_state_pub.pose.orientation = quat;
            ROS_INFO("roll: [%f], pitch: [%f], new_yaw: [%f]", roll, pitch, new_yaw);
            
            // ROS_INFO("x: [%f], y: [%f], z: [%f], w: [%f]", model_state_pub.pose.orientation.x, model_state_pub.pose.orientation.y, model_state_pub.pose.orientation.z, model_state_pub.pose.orientation.w);
            



            

            // ROS_INFO("x: [%f], y: [%f], rot: [%f]", model_state_pub.pose.position.x, model_state_pub.pose.position.y, model_state_pub.pose.orientation.z);
            move_publisher.publish(model_state_pub);
            ros::spinOnce();
            loop_rate.sleep();
            time_ms += 1;
        }
    }
    else if(def_frame == coord::ROBOT)
    {
        model_state_pub.twist.linear.x= 0.02; //0.02: 2cm/sec
        model_state_pub.twist.linear.y= 0.0;
        model_state_pub.twist.linear.z= 0.08;
        
        model_state_pub.twist.angular.x= 0.0;
        model_state_pub.twist.angular.y= 0.0;
        model_state_pub.twist.angular.z= 0.0;

        model_state_pub.reference_frame = "base";

        while(ros::ok())
        {
            move_publisher.publish(model_state_pub);
            loop_rate.sleep();
        }
    }
    
}