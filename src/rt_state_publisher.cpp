#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <robby_track2/pwmDirectional2.h>

// SchmittTrigger
class SchmittTrigger 
{
  enum triggerState {stUpper, stLower, stUndefined};
  // public
  public:
    SchmittTrigger (const float lower_bound, const float upper_bound); // constructor
    void setBounds (const float lower_bound, const float upper_bound); // set new bounds
    void setState (const float value); // get state according to value
    long getStateChanges (); // get current number of state changes
  // private
  private:
    triggerState state_; // state of trigger 
    float lower_bound_;  // lower bound of trigger
    float upper_bound_;  // upper bound of trigger
    long num_state_changes_; // number of state changes
}; // end class SchmittTrigger

SchmittTrigger::SchmittTrigger(const float lower_bound, const float upper_bound) 
{
  state_ = stUndefined;
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
  num_state_changes_ = 0;
}

void SchmittTrigger::setBounds (const float lower_bound, const float upper_bound)
{
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

void SchmittTrigger::setState (const float value)
{
  switch (state_) 
  {
    case stUndefined: 
    {
      if ((lower_bound_ + upper_bound_) / 2.0 > value)
      {
        state_ = stLower;
      }
      else 
      {
        state_ = stUpper;
      }
    }
    case stLower: if (value > upper_bound_) 
    {
      state_ = stUpper;
      num_state_changes_++;
    }
    case stUpper: if (value < lower_bound_) 
    {
      state_ = stLower;
      num_state_changes_++;
    }
  }
}

long SchmittTrigger::getStateChanges ()
{
  return num_state_changes_;
}

// SensorWheel, i.e. infrared sensor against wheel with cutouts
class SensorWheel
{
  public:
    // constructor
    SensorWheel (const int num_cuts, 
                 const float wheel_diameter, 
                 const float lower_bound, 
                 const float upper_bound)
    :
    num_cuts_(num_cuts),
    wheel_diameter_(wheel_diameter),
    wheel_state_(lower_bound, upper_bound),
    old_num_state_changes_(0)
    {}
  private:
    const int num_cuts_; // number of cuts in sensor wheel
    const float wheel_diameter_; // diameter of wheel for velocity calculation 
    SchmittTrigger wheel_state_; // state of wheel depending history of sensor information
    long old_num_state_changes_; // old number of state changes, needed to calculate velocity
  public:
    void updateState (const float value); // updates state given value
    float getDistance(); // returns distance travelled since last call 
};

void SensorWheel::updateState (const float value)
{
  wheel_state_.setState(value);
}

float SensorWheel::getDistance ()
{
  float distance; // [m] distance travelled  
  
  // caculate distance travelled 
  distance = (wheel_state_.getStateChanges() - old_num_state_changes_) 
    / (num_cuts_ * 2.0) * wheel_diameter_;

  // save this call
  old_num_state_changes_ = wheel_state_.getStateChanges();

  return distance;
}

class RobbyTrack
{
  public:
    RobbyTrack (const int num_cuts, 
                 const float wheel_diameter, 
                 const float lower_bound_left, 
                 const float upper_bound_left,
                 const float lower_bound_right, 
                 const float upper_bound_right,
                 const int min_voltage,
                 const int max_voltage,
                 const float min_velocity,
                 const float max_velocity,
                 const float track_distance)
    :
    wheel_left_(num_cuts, wheel_diameter, lower_bound_left, upper_bound_left),
    wheel_right_(num_cuts, wheel_diameter, lower_bound_right, upper_bound_right),
    min_voltage_(min_voltage),
    max_voltage_(max_voltage),
    min_velocity_(min_velocity),
    max_velocity_(max_velocity),
    track_distance_(track_distance)
    {
      odom_trans_.header.frame_id = "odom";
      odom_trans_.child_frame_id = "base_link";
    }
    void velocityCallback(const geometry_msgs::Twist& vel); // velocity callback for command velocity
//    void sensorCallback(...); // sensor callback for sensor wheel information
    // estimation of position & velocity from wheel sensors
    void updateJointsAndOdom(const float dT, ros::Time current_time); 
    const geometry_msgs::TransformStamped& odom_trans() {return odom_trans_;}; // get odom_trans
    const sensor_msgs::JointState& joint_state() {return joint_state_;}; // get joint_state
    const nav_msgs::Odometry& odom() {return odom_;} ; // get odom
    const robby_track2::pwmDirectional2& pwm() {return pwm_;} ; // get odom
  private:
    // absolute position
    float x_; // absolute x-position [m]
    float y_; // absolute y-position [m]
    float th_; // absolute orientation [rad]
    // broadcaster
    tf::TransformBroadcaster odom_broadcaster_;  // odom
    // messages
    geometry_msgs::TransformStamped odom_trans_; // transformation of odom
    sensor_msgs::JointState joint_state_; // joint state
    nav_msgs::Odometry odom_;  // odometry information
    robby_track2::pwmDirectional2 pwm_;
    // wheels
    SensorWheel wheel_left_; // left wheel
    SensorWheel wheel_right_; // right wheel
    // motor
    const int min_voltage_; // for motors
    const int max_voltage_; // for motors
    const float min_velocity_; // for tracks in [m/s]
    const float max_velocity_;  // for tracks in [m/s]
    // tracks
    const float track_distance_; // distance between tracks [m]
    // functions
    int voltageFromVelocity (const float velocity); // returns voltage level for desired velocity
};

int RobbyTrack::voltageFromVelocity (const float velocity)
{
  int voltage_range;
  float velocity_range;
  float used_velocity;

  voltage_range = max_voltage_ - min_voltage_;
  velocity_range = max_velocity_ - min_velocity_;

  if (abs(velocity) < min_velocity_) {
    return 0;
  } else {
    used_velocity = std::min(max_velocity_, float(abs(velocity)));
    return min_voltage_ + (used_velocity - min_velocity_) * voltage_range / velocity_range;
  }
}

// veloctiy callback for command velocity
void RobbyTrack::velocityCallback(const geometry_msgs::Twist& vel)
{
  float vl; // left track velocity [m/s]
  float vr; // right track velocity [m/s]

  float track_velocity_left; // left track velocity [m/s]
  float track_velocity_right; // right track velocity [m/s]
  float voltage_left; // voltage level of left motor 
  float voltage_right; // voltage level of right motor

  int signL = 1;
  int signR = 1;
  
  // rough estimation only valid for small angles
  track_velocity_left = vel.linear.x - track_distance_ / 2.0 * vel.angular.z; 
  // rough estimation only valid for small angles
  track_velocity_right = 2.0 * vel.linear.x - vl; 

  // left motor actuation
  voltage_left = voltageFromVelocity(track_velocity_left);
  // right motor actuation
  voltage_right = voltageFromVelocity(track_velocity_right);

  // write to pwm
  if (track_velocity_left < 0.0) {signL = -1;}; 
  if (track_velocity_right < 0.0) {signR = -1;}; 
  pwm_.pwmDirection1 = voltage_left * signL;
  pwm_.pwmDirection2 = voltage_right * signR;

}

//void RobbyTrack::sensorCallback(...)
//{
  // udpate sensor
//}

void RobbyTrack::updateJointsAndOdom(const float dT, ros::Time current_time)
{
  // distances
  double dl = 0.0; // left wheel distance[m]
  double dr = 0.0; // right wheel distance [m]
  double ds = 0.0; // robot travelled [m]
  double dth = 0.0; // angular change [rad]
  double dx = 0.0; // change in x-direction [m]
  double dy = 0.0; // change in y-direction [m]

  // velocities
  double vx = 0.0; // velocity in x [m/s]
  double vy = 0.0; // velocity in y [m/s]
  double vth = 0.0; // radial velocity [rad/s]

  // servos
  double servo1Angle=0;

  // calculate robot distance travelled in its coordinate frame
  dl = wheel_left_.getDistance();
  dr = wheel_right_.getDistance();
  ds = (dl + dr) / 2.0; 
  dth = (dr - dl) / track_distance_; 
  dx = ds * cos(dth / 2.0); 
  dy = ds * sin(dth / 2.0); 
  
  // calculate robot velocity in its coordinate frame
  vx = dx / dT;
  vy = dy / dT;
  vth = dth / dT;

  // update position in absolute coordinate frame
  x_ += dx * cos(th_) - dy * sin(th_);
  y_ += dx * sin(th_) + dy * cos(th_);
  th_ += dth;

  //update joint_state
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.name.resize(2);
  joint_state_.position.resize(2);
  joint_state_.name[0] ="base_to_head1";
  joint_state_.position[0] = servo1Angle;
  joint_state_.name[1] ="servo1_to_servo2";
  joint_state_.position[1] = servo1Angle;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

  // write to odom
  odom_trans_.header.stamp = current_time;
  odom_trans_.transform.translation.x = x_;
  odom_trans_.transform.translation.y = y_;
  odom_trans_.transform.translation.z = 0.0;
  odom_trans_.transform.rotation = odom_quat;

  //next, we'll publish the odometry message over ROS
  odom_.header.stamp = current_time;
  odom_.header.frame_id = "odom";

  //set the position
  odom_.pose.pose.position.x = x_;
  odom_.pose.pose.position.y = y_;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation = odom_quat;

  //set the velocity
  odom_.child_frame_id = "base_link";
  odom_.twist.twist.linear.x = vx;
  odom_.twist.twist.linear.y = vy;
  odom_.twist.twist.angular.z = vth;
}


// main
int main(int argc, char** argv) {
    // init
    ros::init(argc, argv, "rt_state_publisher");
    ros::NodeHandle nh;

    // robby track
    RobbyTrack robby_track(10, 0.036, 500, 600, 550, 650, 100, 255, 0.01, 0.1, 0.09);

    // subscriber for command velocity
    ros::Subscriber velocity_sub = nh.subscribe("robby_track_1/cmd_vel", 1000, 
      &RobbyTrack::velocityCallback, &robby_track);

    // subscriber for velocity from arduino
    //ros::Subscriber velocity_sub = n.subscribe("robby_track_1/velocity", 1000, velocityCallback);

    // publisher
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher motor_pub = nh.advertise<robby_track2::pwmDirectional2>("motor", 50);

    // broadcaster
    tf::TransformBroadcaster odom_broadcaster;
   
    // rate
    ros::Rate loop_rate(30);

    // time
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (ros::ok()) {
      ros::spinOnce();               // check for incoming messages
      current_time = ros::Time::now();
      double dT = (current_time - last_time).toSec();     
      if (dT > 0.3) 
      {
	robby_track.updateJointsAndOdom(dT, current_time);

        //send the joint state and transform
        joint_pub.publish(robby_track.joint_state());
        odom_broadcaster.sendTransform(robby_track.odom_trans());

        //publish the message
        odom_pub.publish(robby_track.odom());
      }
      
      // publish motor voltage
      motor_pub.publish(robby_track.pwm());

      // time
      last_time = current_time;


      // This will adjust as needed per iteration
      loop_rate.sleep();
    }

    return 0;
}
