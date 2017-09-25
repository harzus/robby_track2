#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

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
    ros::init(argc, argv, "rt_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    // subscriber for velocity
    ros::Subscriber velocity_sub = n.subscribe("robby_track_1/velocity", 1000, velocityCallback);

    const double degree = M_PI/180;

    // robot state
    double servo1Angle=0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="servo1_base_to_head";
        joint_state.position[0] = servo1Angle;
        joint_state.name[1] ="servo1_to_servo2";
        joint_state.position[1] = servo1Angle;


        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x += twist_msg_velocity.linear.x;
        odom_trans.transform.translation.y += twist_msg_velocity.linear.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(twist_msg_velocity.angular.z);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        // servo1Angle += degree/4;
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
