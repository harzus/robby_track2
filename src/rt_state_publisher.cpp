#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>

//--- subcriber---
geometry_msgs::Twist twist_msg_velocity; // estimated robot velocities, linear & angular

float a=1;

// veloctiy callback
void velocityCallback(const geometry_msgs::Twist& vel)
{
    ROS_INFO("velocityCallback triggered");
    twist_msg_velocity = vel;
}

// main
int main(int argc, char** argv) {
    // init
    ros::init(argc, argv, "rt_state_publisher");
    ros::NodeHandle n;

    // publisher
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    // broadcaster
    tf::TransformBroadcaster joint_broadcaster; // joints
    tf::TransformBroadcaster odom_broadcaster;  // odom
    ros::Rate loop_rate(30);

    // subscriber for velocity
    ros::Subscriber velocity_sub = n.subscribe("robby_track_1/velocity", 1000, velocityCallback);

    const double degree = M_PI/180;

    // robot state
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    // servos
    double servo1Angle=0;

    // time
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="base_to_head1";
        joint_state.position[0] = servo1Angle;
        joint_state.name[1] ="servo1_to_servo2";
        joint_state.position[1] = servo1Angle;

        //--- update transform----

        // get velocities
        vx = twist_msg_velocity.linear.x;
        vy = twist_msg_velocity.linear.y;
        vth = twist_msg_velocity.angular.z;

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // write to odom
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the joint state and transform
        joint_pub.publish(joint_state);
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        // time
        last_time = current_time;


        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
