#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <robby_track2/pwmDirectional2.h>
#include <cmath>

// SchmittTrigger
class SchmittTrigger 
{
  enum triggerState {stUpper, stLower, stUndefined};
  // public
  public:
    SchmittTrigger (const double lower_bound, const double upper_bound); // constructor
    void setBounds (const double lower_bound, const double upper_bound); // set new bounds
    void setState (const double value); // get state according to value
    long getStateChanges (); // get current number of state changes
  // private
  //private:
    triggerState state_; // state of trigger 
    double lower_bound_;  // lower bound of trigger
    double upper_bound_;  // upper bound of trigger
    long num_state_changes_; // number of state changes
}; // end class SchmittTrigger

SchmittTrigger::SchmittTrigger(const double lower_bound, const double upper_bound)
{
  state_ = stUndefined;
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
  num_state_changes_ = 0;
}

void SchmittTrigger::setBounds (const double lower_bound, const double upper_bound)
{
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

void SchmittTrigger::setState (const double value)
{
  if (state_ == stUndefined)
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
  else if (state_ == stLower)
  {
      if (value > upper_bound_)
      {
        state_ = stUpper;
        num_state_changes_++;
      }
  }
  else if (value < lower_bound_)
  {
    state_ = stLower;
    num_state_changes_++;
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
                 const double wheel_diameter,
                 const double lower_bound,
                 const double upper_bound)
    :
    num_cuts_(num_cuts),
    wheel_diameter_(wheel_diameter),
    wheel_state_(lower_bound, upper_bound),
    old_num_state_changes_(0)
    {}
  //private:
    const int num_cuts_; // number of cuts in sensor wheel
    const double wheel_diameter_; // diameter of wheel for velocity calculation
    SchmittTrigger wheel_state_; // state of wheel depending history of sensor information
    long old_num_state_changes_; // old number of state changes, needed to calculate velocity
  public:
    void updateState (const double value); // updates state given value
    double getDistance(); // returns distance travelled since last call
    long getStateChanges(); // returns total number of state changes
};

void SensorWheel::updateState (const double value)
{
  wheel_state_.setState(value);
}

double SensorWheel::getDistance ()
{
  double distance; // [m] distance travelled

  // caculate distance travelled 
  distance = double(wheel_state_.getStateChanges() - old_num_state_changes_)
    / double(num_cuts_ * 2.0) * wheel_diameter_ * M_PI;

  // save this call
  old_num_state_changes_ = wheel_state_.getStateChanges();

  return distance;
}

long SensorWheel::getStateChanges()
{
  return wheel_state_.getStateChanges();
}

class ServoControl
{
  public:
    // constructor
    ServoControl (const int min_pos, const int max_pos, const int offset);
    // set target angle
    void setTarget(const int target);
    // get corrected angle for servo [°]
    int correctedServo();
    // get corrected target for servo [rad]
    double correctedTarget();
  private:
    int min_pos_; // minimum allowed postion [°]
    int max_pos_; // maximum allowed postion [°]
    int offset_; // offset [°]
    int target_; // target angle  [°]
    int internal_target_; // internal target angle corrected with constraints [°]
};

void ServoControl::setTarget(const int target)
{
  target_ = target;
  // convert to internal target, watching limits
  internal_target_ = std::max(std::min(offset_ + target, max_pos_), min_pos_);
}

int ServoControl::correctedServo()
{
  return internal_target_;
}

double ServoControl::correctedTarget()
{
  int corrected_target;

  // check for changes
  corrected_target = internal_target_ - offset_;

  // return in rad
  return double(corrected_target) / 180.0 * M_PI;
}


ServoControl::ServoControl (const int min_pos, const int max_pos, const int offset)
{
  min_pos_ = min_pos;
  max_pos_ = max_pos;
  offset_ = offset;
  setTarget(0);
}



class RobbyTrack
{
  public:
    RobbyTrack (const int num_cuts,
                const double wheel_diameter,
                const double lower_bound_left,
                const double upper_bound_left,
                const double lower_bound_right,
                const double upper_bound_right,
                const int min_voltage,
                const int max_voltage,
                const double min_velocity,
                const double max_velocity,
                const double track_distance)
    :
    wheel_left_(num_cuts, wheel_diameter, lower_bound_left, upper_bound_left),
    wheel_right_(num_cuts, wheel_diameter, lower_bound_right, upper_bound_right),
    min_voltage_(min_voltage),
    max_voltage_(max_voltage),
    min_velocity_(min_velocity),
    max_velocity_(max_velocity),
    track_distance_(track_distance),
    servoHor_(10, 150, 74),
    servoVert_(10, 115, 95)
    {
      odom_trans_.header.frame_id = "odom";
      odom_trans_.child_frame_id = "base_link";
      cmd_vel_received_ = false;
      servo_received_ = true;
      servo_automated_ = false;
      sign_left_ = 1;
      sign_right_ = 1;
      servo_auto_target_.pwmDirection1 = 0;
      servo_auto_target_.pwmDirection2 = 0;
      scan_direction_ = 1;
    }
    // true, if command velocity just received
    bool cmd_vel_received_; 
    // true, if servo command just received
    bool servo_received_; 
    // true, if automatic servo control is activated
    bool servo_automated_;
    // velocity callback for command velocity
    void velocityCallback(const geometry_msgs::Twist& vel); 
    // sensor callback for sensor wheel information
    void sensorCallback(const robby_track2::pwmDirectional2& msg);
    // callback for servo position
    void servoCallback(const robby_track2::pwmDirectional2& msg);
    // automatically move servo for scan
    void scanServo();
    // estimation of position & velocity from wheel sensors
    void updateJointsAndOdom(const double dT, ros::Time current_time);
    // get odom_trans
    geometry_msgs::TransformStamped& odom_trans() {return odom_trans_;}; 
    // get joint_state
    sensor_msgs::JointState& joint_state() {return joint_state_;}; 
    // get odom
    nav_msgs::Odometry& odom() {return odom_;} ; 
    // get odom
    robby_track2::pwmDirectional2& pwm() {return pwm_;} ; 
    // get state changes for debug
    robby_track2::pwmDirectional2& getStateChanges() {
      return sensorStateChanges_;
    } ; 
    // get servo_sent
    robby_track2::pwmDirectional2& servo_sent() {return servo_sent_;} ; 
  private:
    // absolute position
    double x_; // absolute x-position [m]
    double y_; // absolute y-position [m]
    double th_; // absolute orientation [rad]
    // broadcaster
    tf::TransformBroadcaster odom_broadcaster_;  // odom
    // messages
    geometry_msgs::TransformStamped odom_trans_; // transformation of odom
    sensor_msgs::JointState joint_state_; // joint state
    nav_msgs::Odometry odom_;  // odometry information
    robby_track2::pwmDirectional2 pwm_; // pwm command for for motors
    robby_track2::pwmDirectional2 sensorStateChanges_; // debug, returns number of state changes
    robby_track2::pwmDirectional2 servo_sent_; // servo message to be sent, correct by zero-offset
    robby_track2::pwmDirectional2 servo_auto_target_; // automatic servo target
    // wheels
    SensorWheel wheel_left_; // left wheel
    SensorWheel wheel_right_; // right wheel
    int sign_left_; // direction of left track, 1: forward, -1: backward
    int sign_right_; // direction of right track, 1: forward, -1: backward
    // motor
    const int min_voltage_; // for motors
    const int max_voltage_; // for motors
    const double min_velocity_; // for tracks in [m/s]
    const double max_velocity_;  // for tracks in [m/s]
    // tracks
    const double track_distance_; // distance between tracks [m]
    // servos
    ServoControl servoHor_; // horizontal servo
    ServoControl servoVert_; // vertical servo
    robby_track2::pwmDirectional2 servos_auto_target_; // current target in automatic mode
    int scan_direction_; // scan direction of servo in automatic mode
    // functions
    int voltageFromVelocity (const double velocity); // returns voltage level for desired velocity
};

int RobbyTrack::voltageFromVelocity (const double velocity)
{
  int voltage_range;
  double velocity_range;
  double used_velocity;

  voltage_range = max_voltage_ - min_voltage_;
  velocity_range = max_velocity_ - min_velocity_;

  if (std::abs(velocity) < min_velocity_) {
    return 0;
  } else {
    used_velocity = std::min(max_velocity_, double(std::abs(velocity)));
    return min_voltage_ + double(used_velocity - min_velocity_) * double(voltage_range / velocity_range);
  }
}

void RobbyTrack::velocityCallback(const geometry_msgs::Twist& vel)
{
  double track_velocity_left; // left track velocity [m/s]
  double track_velocity_right; // right track velocity [m/s]
  double voltage_left; // voltage level of left motor
  double voltage_right; // voltage level of right motor
  int vel_sign_left; // diretion of command velocity
  int vel_sign_right; // diretion of command velocity

  // rough estimation only valid for small angles
  track_velocity_left = vel.linear.x - track_distance_ / 2.0 * vel.angular.z; 
  // rough estimation only valid for small angles
  track_velocity_right = 2.0 * vel.linear.x - track_velocity_left;

  // left motor actuation
  voltage_left = voltageFromVelocity(track_velocity_left);
  // right motor actuation
  voltage_right = voltageFromVelocity(track_velocity_right);

  // write to pwm
  if (track_velocity_left < 0.0) {
    vel_sign_left = -1;
  } else {
    vel_sign_left = 1;
  }
  if (track_velocity_right < 0.0) {
    vel_sign_right = -1;
  } else {
    vel_sign_right = 1;
  }
  pwm_.pwmDirection1 = voltage_left * vel_sign_left;
  pwm_.pwmDirection2 = voltage_right * vel_sign_right;
  // set flag
  cmd_vel_received_ = true;
}

void RobbyTrack::sensorCallback(const robby_track2::pwmDirectional2& msg)
{
  wheel_left_.updateState(std::abs(msg.pwmDirection1));
  wheel_right_.updateState(std::abs(msg.pwmDirection2));

  // save current direction, no change when zero
  if (msg.pwmDirection1 < 0) {
    sign_left_ = -1;
  } else if (msg.pwmDirection1 > 0){
    sign_left_ = 1;
  }
  if (msg.pwmDirection2 < 0) {
    sign_right_ = -1;
  } else if (msg.pwmDirection2 > 0){
    sign_right_ = 1;
  }
}

void RobbyTrack::servoCallback(const robby_track2::pwmDirectional2& msg)
{
  // check for automated modus
  if ((msg.pwmDirection1 == 999) || (msg.pwmDirection2 == 999))
  {
    servo_automated_ = true;
  } else {
    servo_automated_ = false;
  } 

  if (not servo_automated_)
  {
    // save message
    servoHor_.setTarget(msg.pwmDirection1);
    servoVert_.setTarget(msg.pwmDirection2);
    // correct offset
    servo_sent_.pwmDirection1 = servoHor_.correctedServo();
    servo_sent_.pwmDirection2 = servoVert_.correctedServo();
    // set flag
    servo_received_ = true;
  }
}

void RobbyTrack::scanServo()
{
  const int minAngle=-60;
  const int maxAngle=60;

  if (servo_auto_target_.pwmDirection1 >= maxAngle)
  {
    scan_direction_ = -1;
  }
  if (servo_auto_target_.pwmDirection1 <= minAngle)
  {
    scan_direction_ = 1;
  }
  // change direction
  servo_auto_target_.pwmDirection1 += scan_direction_;
  // callback
  // save message
  servoHor_.setTarget(servo_auto_target_.pwmDirection1);
  servoVert_.setTarget(servo_auto_target_.pwmDirection2);
  // correct offset
  servo_sent_.pwmDirection1 = servoHor_.correctedServo();
  servo_sent_.pwmDirection2 = servoVert_.correctedServo();
  // set flag
  servo_received_ = true;
}

void RobbyTrack::updateJointsAndOdom(const double dT, ros::Time current_time)
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

  // calculate robot distance travelled in its coordinate frame
  dl = double(sign_left_) * wheel_left_.getDistance(); 
  dr = double(sign_right_) * wheel_right_.getDistance(); 
  // debug
  //sensorStateChanges_.pwmDirection1 = double(wheel_left_.wheel_state_.getStateChanges() - wheel_left_.old_num_state_changes_)
  //    / double(wheel_left_.num_cuts_ * 2.0)* wheel_left_.wheel_diameter_*1e6;
  sensorStateChanges_.pwmDirection1 += dl*1e3;
  sensorStateChanges_.pwmDirection2 += dr*1e3;
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
  joint_state_.position[0] = servoHor_.correctedTarget();
  joint_state_.name[1] ="servo1_to_servo2";
  joint_state_.position[1] = servoVert_.correctedTarget();

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
    ros::Subscriber velocity_sub = nh.subscribe(
          "robby_track_1/cmd_vel",
          1000,
          &RobbyTrack::velocityCallback,
          &robby_track
          );

    // subscriber for track sensros from arduino
    ros::Subscriber track_sub = nh.subscribe(
          "/robby_track_1/sensor_tracks",
          1000,
          &RobbyTrack::sensorCallback,
          &robby_track
          );

    // subscriber for servo position
    ros::Subscriber servo_sub = nh.subscribe(
          "/robby_track_1/servo",
          1000,
          &RobbyTrack::servoCallback,
          &robby_track
          );

    // publisher
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher motor_pub = nh.advertise<robby_track2::pwmDirectional2>("/robby_track_1/motor_pwm", 50);
    ros::Publisher sensor_pub = nh.advertise<robby_track2::pwmDirectional2>("/robby_track_1/sensor_count", 50);
    ros::Publisher servo_pub = nh.advertise<robby_track2::pwmDirectional2>("/robby_track_1/servo_corr", 50);
    // broadcaster
    tf::TransformBroadcaster odom_broadcaster;
   
    // rate
    ros::Rate loop_rate(30);

    // time
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (ros::ok()) {
      current_time = ros::Time::now();
      double dT = (current_time - last_time).toSec();     
      if (dT > 0.1)
      {
        // joints and odom
        robby_track.updateJointsAndOdom(dT, current_time);

        //send the joint state and transform
        joint_pub.publish(robby_track.joint_state());
        odom_broadcaster.sendTransform(robby_track.odom_trans());

        //publish the message
        odom_pub.publish(robby_track.odom());

        // debug, publish state changes
        sensor_pub.publish(robby_track.getStateChanges());

        // auto scan
        if (robby_track.servo_automated_) {robby_track.scanServo();}

        // time
        last_time = current_time;
      }
      
      if (robby_track.cmd_vel_received_== true)
      {
        // publish motor voltage
        motor_pub.publish(robby_track.pwm());
        robby_track.cmd_vel_received_ = false;
      }

      if (robby_track.servo_received_== true)
      {
        // publish servo position
        servo_pub.publish(robby_track.servo_sent());
        robby_track.servo_received_ = false;
      }

      // messages
      ros::spinOnce();

      // This will adjust as needed per iteration
      loop_rate.sleep();
    }

    return 0;
}
