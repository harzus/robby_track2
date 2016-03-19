#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <Arduino.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world2";

/*To control the rover, Copy and paste the code below into the Arduino software*/
int E1 = 6; //M1 Speed Control
int E2 = 5; //M2 Speed Control
int M1 = 8; //M1 Direction Control
int M2 = 7; //M2 Direction Control

// start one side of the robot
void messageCb( const std_msgs::Empty& toggle_msg){
  analogWrite (E1,255);
  digitalWrite(M1,LOW);
}

// subcriber for ping message
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


void setup()
{
  // init
  nh.initNode();
  // advertise
  nh.advertise(chatter);
  // subscribe
  nh.subscribe(sub);
  int i;
  for(i=5;i<=8;i++)
  pinMode(i, OUTPUT);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}

//test
/*#include <ros.h>
#include <std_msgs/String.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
//#include <std_msgs/Float64.h>
#include <Arduino.h>

ros::NodeHandle nh;

int count;
//float lin_vel;
//float ang_vel;
//ros::WallTime last_command_time;

void messageCb( const std_msgs::Empty& toggle_msg){
  count++;
}


//void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
//{
//  last_command_time = ros::WallTime::now();
//  lin_vel = vel->linear.x;
//  ang_vel = vel->angular.z;
//}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
//ros::Subscriber<geometry_msgs::Twist> rtTwist("robby_track_1/cmd_vel", 1);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello1[15] = "hello world 1!";
char hello2[15] = "hello world 2!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  if ((count % 2) == 0) {
    str_msg.data = hello1;
  } else {
    str_msg.data = hello2;
  }
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
*/
