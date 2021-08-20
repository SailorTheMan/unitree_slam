/*
  Node subscribes to odom_in topic reads data and publishes it to odom_out topic,
  but with frame ids changed to base and odom.
  Also it broadcasts transform between base and odom.
*/

#include <odom_handler.h>

OdomHandlerNode::OdomHandlerNode()
{
  bool init_result = init();
  ROS_ASSERT(init_result);
}

OdomHandlerNode::~OdomHandlerNode()
{
}

bool OdomHandlerNode::init()
{
  publisher_  = nh_.advertise<nav_msgs::Odometry>("odom_out", 10);
  subscriber_ = nh_.subscribe("odom_in", 150, &OdomHandlerNode::msgCallback, this);

  return true;
}

void OdomHandlerNode::msgCallback(const nav_msgs::OdometryConstPtr odom_in)
{
  if (last_published_time_.isZero() || odom_in->header.stamp > last_published_time_)
  {

    last_published_time_ = odom_in->header.stamp;

    geometry_msgs::TransformStamped tf;

    tf.header.stamp = odom_in->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base";

    tf.transform.translation.x = odom_in->pose.pose.position.x;
    tf.transform.translation.y = odom_in->pose.pose.position.y;
    tf.transform.translation.z = odom_in->pose.pose.position.z;

    tf.transform.rotation.x = odom_in->pose.pose.orientation.x;
    tf.transform.rotation.y = odom_in->pose.pose.orientation.y;
    tf.transform.rotation.z = odom_in->pose.pose.orientation.z;
    tf.transform.rotation.w = odom_in->pose.pose.orientation.w;

    tf_broadcaster.sendTransform(tf);

    nav_msgs::Odometry odom_out;

    odom_out.header.stamp = odom_in->header.stamp;
    odom_out.header.frame_id = "odom";
    odom_out.child_frame_id = "base";

    odom_out.pose = odom_in->pose;
    odom_out.twist = odom_in->twist;

    publisher_.publish(odom_out);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "flat_world_imu_node");

  OdomHandlerNode flat_world_imu_node;

  ros::spin();

  return 0;
}
