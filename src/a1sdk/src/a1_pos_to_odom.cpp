

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

class LastValues{
public:   
    double lastX;
    double lastY;

    double lastFixedX;
    double lastFixedY;

    double nakopX;
    double nakopY;

    double velX;
    double velY;

    bool recvFlagIMU;
    bool recvFlagPoint;

    geometry_msgs::Vector3 lastAccels;

    ros::Time lastTimeStamp;
    geometry_msgs::PointStamped lastRecPoint;
    sensor_msgs::Imu lastRecIMU;
    geometry_msgs::Point worldCoords;

    ros::Publisher *odom_pub;
    
    LastValues()
    {
        lastX = 0;
        lastY = 0;
        nakopX = 0;
        nakopY = 0;
        velX = 0;
        velY = 0;

        recvFlagIMU = false;
        recvFlagPoint = false;

        lastFixedX = 0.0;
        lastFixedY = 0.0;

        worldCoords.x = 0.0;
        worldCoords.y = 0.0;

        lastAccels.x = 0.0;
        lastAccels.y = 0.0;
        lastAccels.z = 0.0;


        
    }


    void Update(const geometry_msgs::PointStamped &error_msg, geometry_msgs::PointStamped &fixed_msg)
    {
        if (error_msg.header.stamp < lastTimeStamp)
        {
            nakopX = 0;     // purge nakop on new set of data 
            nakopY = 0;
            lastX = error_msg.point.x;
            lastY = error_msg.point.y;

            worldCoords.x = 0.0;
            worldCoords.y = 0.0;
        }

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
        lastTimeStamp = error_msg.header.stamp;
    }

    geometry_msgs::PointStamped ToWorld(const geometry_msgs::PointStamped local_msg)
    {

        
    }

    void Fuse()     // kinda
    {
        if (! recvFlagPoint && recvFlagIMU)
        return;

        nav_msgs::Odometry odom_msgs;
        geometry_msgs::PointStamped fixed_msg;

        Update(lastRecPoint, fixed_msg);

        //
        geometry_msgs::Quaternion msg = lastRecIMU.orientation;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg, quat);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        //std::cout << yaw << std::endl;
        //std::cout << fixed_msg.point << std::endl;
        worldCoords.x += (fixed_msg.point.x - lastFixedX) * cos(yaw) - (fixed_msg.point.y - lastFixedY) * sin(yaw);  //.001 * (sin(phi) * state.forwardSpeed * fwdCoef + cos(phi) * state.sideSpeed * 0.4);
        worldCoords.y += (fixed_msg.point.x - lastFixedX) * sin(yaw) + (fixed_msg.point.y - lastFixedY) * cos(yaw);
        worldCoords.z = fixed_msg.point.z;
        // /std::cout << worldCoords << std::endl;

        double dt = lastRecIMU.header.stamp.toSec() - lastTimeStamp.toSec();

        odom_msgs.pose.pose.position = worldCoords;
        odom_msgs.pose.pose.orientation = lastRecIMU.orientation;
        odom_msgs.twist.twist.angular = lastRecIMU.angular_velocity;

        velX += (lastRecIMU.linear_acceleration.x - lastAccels.x) * dt;
        velY += (lastRecIMU.linear_acceleration.y - lastAccels.y) * dt;
        odom_msgs.twist.twist.linear.x = velX;
        odom_msgs.twist.twist.linear.y = velY;
        odom_msgs.twist.twist.linear.z = 0.0; // not enough data

        odom_msgs.header = fixed_msg.header;

        lastFixedX = fixed_msg.point.x;
        lastFixedY = fixed_msg.point.y;
        lastAccels = lastRecIMU.linear_acceleration;

        Publish(odom_msgs);
        recvFlagPoint = false;
        recvFlagIMU = false;
        
    }

    void Publish(nav_msgs::Odometry odom_msgs)
    {
        odom_pub->publish(odom_msgs);
    }
};

LastValues LV;


ros::Publisher odom_pub;
ros::Subscriber error_sub;
ros::Subscriber imu_sub;



void MsgCallback(const geometry_msgs::PointStamped error_msg)
{
    LV.lastRecPoint = error_msg;
    LV.recvFlagPoint = true;

    LV.Fuse();
}    

void imuCallback(const sensor_msgs::Imu imu_msg)
{
    LV.lastRecIMU = imu_msg;
    LV.recvFlagPoint = true;

    LV.Fuse();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    
    error_sub = n.subscribe("odom_error", 1000, MsgCallback);
    imu_sub = n.subscribe("IMU_data", 1000, imuCallback);
    odom_pub = n.advertise<nav_msgs::Odometry>("fused_odometry", 1000);

    LV.worldCoords.x = 0.0;
    LV.worldCoords.y = 0.0;
    LV.odom_pub = &odom_pub;

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for odometry (/odom_error) and IMY (/IMU_data)");
    ros::spin();
    return 0;
}