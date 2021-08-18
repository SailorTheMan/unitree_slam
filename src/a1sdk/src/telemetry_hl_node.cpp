

    // TODO: 
    // Need to visualize:
    // > each leg position              \/
    // > each foot sensor vector        \/
    // > IMU data                       \/
    // > body height                    \/
    // > foot/forward speed             \both/

    // FIXME:
    // > incorrect message stamps       \/
    // > rewrite imu msg filling through memcpy
    


#include "unitree_legged_sdk/unitree_legged_sdk.h"
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

#define EXEC_EXAMPLE false

using namespace UNITREE_LEGGED_SDK;

// FR FL RR RL
std::string footFrames[4] = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};

class ROS_Publishers
{
public:
    ros::Publisher *chatter;
    ros::Publisher *leg_force_pub[4];
    ros::Publisher *leg_velocity_pub[4];
    ros::Publisher *imu_pub;
    ros::Publisher *leg_pose;
    ros::Publisher *errPos_pub;
    ros::Publisher *naiveOdom_pub;
    int seq;
};

class NaiveOdometry         // WORK IN PROGRESS
{
public:
    geometry_msgs::Point position;
    float worldX;
    float worldY;
    float phi;                      // rads ??
    const float dt = 0.02; 

    geometry_msgs::Point Update(HighState& state)
    {
        double fwdCoef = state.forwardSpeed > 0 ? 1 : 0.7;
        phi = phi + state.rotateSpeed * 120 * 3.1415 / 180 * dt;
        if (phi > 3.1415) phi = -3.1415;
        if (phi < -3.1415) phi = 3.1415;

        worldX += 0.001 * (cos(phi) * state.forwardSpeed * fwdCoef - sin(phi) * state.sideSpeed * 0.4);  // or smthng like that
        worldY += 0.001 * (sin(phi) * state.forwardSpeed * fwdCoef + cos(phi) * state.sideSpeed * 0.4); 

        position.x = worldX;
        position.y = worldY;
        position.z = state.bodyHeight;

        return this->position;
    }
    
    
};

float lastForwVelocity = 0.0;
float lastSideVelocity = 0.0;
NaiveOdometry NOdom;


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl(ROS_Publishers rospub);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void fillImuData(HighState &state, sensor_msgs::Imu &imuData, ROS_Publishers &rospub)
{
    //memcpy?
    imuData.linear_acceleration.x = state.imu.accelerometer[0];
    imuData.linear_acceleration.y = state.imu.accelerometer[1];
    imuData.linear_acceleration.z = state.imu.accelerometer[2];

    imuData.angular_velocity.z = state.rotateSpeed;
    // i dont remember why i did this
    imuData.linear_acceleration.y = (state.forwardSpeed - lastForwVelocity) / 0.02;         // TODO: pass dt here
    imuData.linear_acceleration.x = (state.sideSpeed - lastSideVelocity) / 0.02;
 
    lastForwVelocity = state.forwardSpeed;
    lastSideVelocity = state.sideSpeed;
    
    imuData.orientation.w = state.imu.quaternion[0];
    imuData.orientation.x = state.imu.quaternion[1];
    imuData.orientation.y = state.imu.quaternion[2];
    imuData.orientation.z = state.imu.quaternion[3];

    imuData.header.seq = rospub.seq;
    imuData.header.frame_id = "imu_link";
    imuData.header.stamp = ros::Time::now();
}

void fillPolyData(HighState &state, geometry_msgs::PolygonStamped &legPolygon, ROS_Publishers &rospub)
{   
    for (int leg = 0; leg < 4; leg++)
    {
        geometry_msgs::Point32 curLegPoint;
        std::memcpy(&curLegPoint, &state.footPosition2Body[leg], sizeof(Cartesian));
        legPolygon.polygon.points.push_back(curLegPoint);
    }
    legPolygon.header.frame_id = "trunk";
    legPolygon.header.seq = rospub.seq;
    legPolygon.header.stamp = ros::Time::now();
}


void SendToROS(Custom *a1Interface, ROS_Publishers rospub)
{
    HighState state = a1Interface->state;
    // Declaring messages
    std_msgs::String msg;                       // chatter
    geometry_msgs::WrenchStamped legForces[4];  // foot forces
    geometry_msgs::WrenchStamped legVels[4];
    geometry_msgs::PolygonStamped legPolygon;
    geometry_msgs::PointStamped errPos;
    geometry_msgs::PointStamped nOdom;
    sensor_msgs::Imu imuData;
    // Pub mode to /chatter
    msg.data = std::to_string(a1Interface->state.mode);
    // Filling forces and vels for each leg
    // FR FL RR RL
    for (int leg = 0; leg < 4; leg++)
    {
        // Constructing leg forces messages
        legForces[leg].wrench.force.z = state.footForce[leg];       // footForceEst ??
        legForces[leg].header.frame_id = footFrames[leg];
        legForces[leg].header.seq = rospub.seq;
        legForces[leg].header.stamp = ros::Time::now();
        // Constructing leg velocities messages
        legVels[leg].wrench.force.x = state.footSpeed2Body[leg].x;       // not actually a force
        legVels[leg].wrench.force.y = state.footSpeed2Body[leg].y;
        legVels[leg].wrench.force.z = state.footSpeed2Body[leg].z;
        legVels[leg].header.frame_id = footFrames[leg];
        legVels[leg].header.seq = rospub.seq;
        legVels[leg].header.stamp = ros::Time::now();

    }
    fillImuData(state, imuData, rospub);
    // Leg position polygon
    fillPolyData(state, legPolygon, rospub);
    // Filling embedded odometry message
    errPos.point.x = state.forwardPosition;
    errPos.point.y = state.sidePosition;
    errPos.point.z = state.bodyHeight;
    errPos.header.frame_id = "base";
    errPos.header.seq = rospub.seq;
    errPos.header.stamp = ros::Time::now();
    // Filling naive odometry message
    nOdom.point = NOdom.Update(state);
    nOdom.header.frame_id = "base";
    nOdom.header.seq = rospub.seq;
    nOdom.header.stamp = ros::Time::now();
    
    // PUBLISHING
    for (int leg = 0; leg < 4; leg++)
    {
        rospub.leg_force_pub[leg]->publish(legForces[leg]);
        rospub.leg_velocity_pub[leg]->publish(legVels[leg]);
    }
    rospub.chatter->publish(msg);
    rospub.imu_pub->publish(imuData);
    rospub.leg_pose->publish(legPolygon);
    rospub.errPos_pub->publish(errPos);
    rospub.naiveOdom_pub->publish(nOdom);

    rospub.seq++;
    ros::spinOnce();
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

    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.roll  = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;

if (EXEC_EXAMPLE) {
    // 1000 motiontimes ~~ 1 sec
    if(motiontime>1000 && motiontime<1500){
        cmd.mode = 1;
        cmd.roll = 0.5f;
    }

    if(motiontime>1500 && motiontime<2000){
        cmd.mode = 1;
        cmd.pitch = 0.3f;
    }

    if(motiontime>2000 && motiontime<2500){
        cmd.mode = 1;
        cmd.yaw = 0.3f;
    }

    if(motiontime>2500 && motiontime<3000){
        cmd.mode = 1;
        cmd.bodyHeight = -0.3f;
    }

    if(motiontime>3000 && motiontime<3500){
        cmd.mode = 1;
        cmd.bodyHeight = 0.3f;
    }

    if(motiontime>3500 && motiontime<4000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0f;
    }

    if(motiontime>4000 && motiontime<5000){
        cmd.mode = 2;
    }

    if(motiontime>5000 && motiontime<8500){
        cmd.mode = 2;
        cmd.forwardSpeed = 0.1f; // -1  ~ +1
    }

    if(motiontime>8500 && motiontime<12000){
        cmd.mode = 2;
        cmd.forwardSpeed = -0.2f; // -1  ~ +1
    }

    if(motiontime>12000 && motiontime<16000){
        cmd.mode = 2;
        cmd.rotateSpeed = 0.3f;   // turn
    }

    if(motiontime>16000 && motiontime<20000){
        cmd.mode = 2;
        cmd.rotateSpeed = -0.3f;   // turn
    }

    if(motiontime>20000 ){
        cmd.mode = 1;
    }
}
//else cmd.mode = 2;

    if (ros::ok())
        SendToROS(this, rospub);
    udp.SetSend(cmd);
}

int main(int argc, char **argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    NOdom.worldX = 0;
    NOdom.worldY = 0;
    NOdom.phi = 0;

    ros::init(argc, argv, "a1Telemetry");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub;
    //  Debug
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    // Forces
    ros::Publisher FRf_pub = nh.advertise<geometry_msgs::WrenchStamped>("FR_force", 1000);
    ros::Publisher FLf_pub = nh.advertise<geometry_msgs::WrenchStamped>("FL_force", 1000);
    ros::Publisher RRf_pub = nh.advertise<geometry_msgs::WrenchStamped>("RR_force", 1000);
    ros::Publisher RLf_pub = nh.advertise<geometry_msgs::WrenchStamped>("RL_force", 1000);
    // Velocities
    ros::Publisher FRv_pub = nh.advertise<geometry_msgs::WrenchStamped>("FR_vel", 1000);
    ros::Publisher FLv_pub = nh.advertise<geometry_msgs::WrenchStamped>("FL_vel", 1000);
    ros::Publisher RRv_pub = nh.advertise<geometry_msgs::WrenchStamped>("RR_vel", 1000);
    ros::Publisher RLv_pub = nh.advertise<geometry_msgs::WrenchStamped>("RL_vel", 1000);
    // IMU
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 1000);
    // Position 2 Body
    ros::Publisher LegPose_pub = nh.advertise<geometry_msgs::PolygonStamped>("feet_polygon", 1000);
    ros::Publisher errPos_pub = nh.advertise<geometry_msgs::PointStamped>("odom_error", 1000);
    ros::Publisher nOdom_pub = nh.advertise<geometry_msgs::PointStamped>("nodom", 1000);
    
    

    /* ROS structure construction for loop */
    ROS_Publishers rospub;      // a structure to pass into loop control
    rospub.chatter = &chatter_pub;
    rospub.leg_force_pub[0] = &FRf_pub;    // packing all neccessary ros objects together
    rospub.leg_force_pub[1] = &FLf_pub;
    rospub.leg_force_pub[2] = &RRf_pub;
    rospub.leg_force_pub[3] = &RLf_pub;
    rospub.leg_velocity_pub[0] = &FRv_pub;   
    rospub.leg_velocity_pub[1] = &FLv_pub;
    rospub.leg_velocity_pub[2] = &RRv_pub;
    rospub.leg_velocity_pub[3] = &RLv_pub;
    rospub.imu_pub = &IMU_pub;
    rospub.leg_pose = &LegPose_pub;
    rospub.errPos_pub = &errPos_pub;
    rospub.naiveOdom_pub = &nOdom_pub;
    rospub.seq = 0;

    Custom custom(HIGHLEVEL);
    // Custom lowCustom(LOWLEVEL);
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
