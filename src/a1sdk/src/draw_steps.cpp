#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PolygonStamped.h"       // represents leg positions
#include "geometry_msgs/PointStamped.h"


ros::Subscriber fr_sub;
ros::Subscriber fl_sub;
ros::Subscriber rr_sub;
ros::Subscriber rl_sub;
ros::Subscriber poly_sub;

class Forces
{
    // FR FL RR RL
public:
    double legForces[4];
    geometry_msgs::Point32 legPoses[4];
    bool legStatus[4];          // True => On the ground; False => Raised
    std::string frame;

    double contactThresh = 150;    // larger values usually mean ground contact
    double raiseThresh = -0.27;     // lower values usually mean ground contact

    ros::Publisher fr_pub;
    ros::Publisher fl_pub;
    ros::Publisher rr_pub;
    ros::Publisher rl_pub;
/*
    Forces Forces(this)
    {
        
    }
*/
    void updateLegStatus()
    {
        for (int leg = 0; leg < 4; leg++)
        {
            legForces[leg] >= contactThresh && legPoses[leg].z <= raiseThresh ? legStatus[leg] = true : legStatus[leg] = false;
        }
    }

    void publishPoints()
    {
        return;
    }

};

Forces forcesHub;


void FRMsgCb(const geometry_msgs::WrenchStamped force_msg)
{
    forcesHub.legForces[0] = force_msg.wrench.force.z;
}    

void FLMsgCb(const geometry_msgs::WrenchStamped force_msg)
{
    forcesHub.legForces[1] = force_msg.wrench.force.z;
}  

void RRMsgCb(const geometry_msgs::WrenchStamped force_msg)
{
    forcesHub.legForces[2] = force_msg.wrench.force.z;
}  

void RLMsgCb(const geometry_msgs::WrenchStamped force_msg)
{
    forcesHub.legForces[3] = force_msg.wrench.force.z;
}  

void poly_cb(const geometry_msgs::PolygonStamped polygon)
{   
    for (int leg = 0; leg < 4; leg++)
    {
        forcesHub.legPoses[leg] = polygon.polygon.points[leg];
    }
    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "step_drawer");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    fr_sub = n.subscribe("FR_force", 1000,  FRMsgCb);
    fl_sub = n.subscribe("FL_force", 1000,  FLMsgCb);
    rr_sub = n.subscribe("RR_force", 1000,  RRMsgCb);
    rl_sub = n.subscribe("RL_foree", 1000,  RLMsgCb);

    ros::Publisher fr_pub = n.advertise<geometry_msgs::PointStamped>("fr_step", 1000);
    ros::Publisher fl_pub = n.advertise<geometry_msgs::PointStamped>("fl_step", 1000);
    ros::Publisher rr_pub = n.advertise<geometry_msgs::PointStamped>("rr_step", 1000);
    ros::Publisher rl_pub = n.advertise<geometry_msgs::PointStamped>("rl_step", 1000);

    poly_sub = n.subscribe("feet_polygon", 1000, poly_cb);

    int locSeq = 0;

    while (ros::ok())
    {
        forcesHub.updateLegStatus();

        ros::spinOnce();
        loop_rate.sleep();
        locSeq++;
    }

    return 0;
}