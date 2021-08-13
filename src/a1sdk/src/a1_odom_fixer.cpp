

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

class lastValues{
public:   
    double lastX;
    double lastY;
    double nakopX;
    double nakopY;

    void Update(const geometry_msgs::PointStamped &error_msg, geometry_msgs::PointStamped &fixed_msg)
    {
        if (abs(lastX / error_msg.point.x) > 4) {
            nakopX += lastX - error_msg.point.x; 
        }
        if (abs(lastX / error_msg.point.x) > 4) {
            nakopY += lastY - error_msg.point.y;
        }

        fixed_msg.point.x = error_msg.point.x + nakopX;
        fixed_msg.point.y = error_msg.point.y + nakopY;
        fixed_msg.point.z = error_msg.point.z;
        fixed_msg.header = error_msg.header;

        lastX = error_msg.point.x;
        lastY = error_msg.point.y;

        //std::cout << "nakop x:" << nakopX << " nakop y:" << nakopY << std::endl;
        //std::cout << fixed_msg.point.x << std::endl;
    }
};

lastValues LV = {0.0, 0.0, 0.0, 0.0};


ros::Publisher fixed_pub;
ros::Subscriber error_pub;


void MsgCallback(const geometry_msgs::PointStamped error_msg)
{
    geometry_msgs::PointStamped fixed_msg; 

    LV.Update(error_msg, fixed_msg);

    fixed_pub.publish(fixed_msg);
}    



int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    fixed_pub = n.advertise<geometry_msgs::PointStamped>("fixed_odom_error", 1000);
    error_pub = n.subscribe("odom_error", 1000, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for data");
    ros::spin();
    return 0;
}