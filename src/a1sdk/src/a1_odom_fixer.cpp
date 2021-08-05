

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"

class lastValues{
public:   
    double lastX;
    double lastY;
    double nakop;

    void Update(geometry_msgs::PointStamped &error_msg, geometry_msgs::PointStamped &fixed_msg)
    {
        if 


        fixed_msg.header = error_msg.header;
    }
};

lastValues LV = {1.0, 1.0, 1.0};


ros::Publisher fixed_pub;
ros::Subscriber error_pub;


void MsgCallback(const geometry_msgs::PointStamped error_msg)
{
    geometry_msgs::PointStamped fixed_msg; 


    fixed_pub.publish(fixed_msg);
}    



int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    fixed_pub = n.advertise<geometry_msgs::Vector3>("fixed_odom_error", 1000);
    error_pub = n.subscribe("odom_error", 1000, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}