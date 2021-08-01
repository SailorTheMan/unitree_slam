#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"


static sensor_msgs::PointCloud2::ConstPtr& msg_ptr;

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
{
    msg_ptr = message;
    ROS_INFO("got msg: %f", message->header);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_timestamp_node");
    ros::NodeHandle n;

    ros::Subscriber rot_table_sub = n.subscribe("/livox/lidar", 100, lidarCallback);
    ros::Publisher rot_table_pub = n.advertise<sensor_msgs::PointCloud2>("/lidar_sync", 100);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        sensor_msgs::PointCloud2 msg;
        msg.header.seq = msg_ptr->header.seq;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = msg_ptr->header.frame_id;

        msg.height = msg_ptr->height;
        msg.width = msg_ptr->width;

        msg.fields = msg_ptr->fields;
        msg.is_bigendian = msg_ptr->is_bigendian;
        
        rot_table_pub.publish(msg);
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}