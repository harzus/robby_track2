#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <roscpp/Logger.h>
#include <tf/tf.h>
#include <string.h>


#include <Arduino.h>

ros::NodeHandle nh;

// will store last time
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
// interval1 to do something in milliseconds
const long interval1 = 100;
int numIntervals1 = 0;
bool intervalStarted1;
// interval2 to do something in milliseconds
const long interval2 = 300;
int numIntervals2 = 0;
bool intervalStarted2;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world2";

/*To control the rover, Copy and paste the code below into the Arduino software*/
int E1 = 6; //M1 Speed Control
int E2 = 5; //M2 Speed Control
int M1 = 8; //M1 Direction Control
int M2 = 7; //M2 Direction Control

// velocity
geometry_msgs::Twist twist;

// veloctiy callback
void velocityCallback(const geometry_msgs::Twist& vel)
{
  twist = vel;
}

// subscriber for velocity
ros::Subscriber<geometry_msgs::Twist> velocity_sub("robby_track_1/cmd_vel", &velocityCallback);

void setupMotor()
{
  int i;
  for(i=E1;i<=M1;i++)
  pinMode(i, OUTPUT);
}

void stop() //Stop
{
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
}

void forward(char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}

void reverse (char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}

void left (char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}

void right (char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}

void loopMotor()
{
    int leftspeed = 255; //255 is maximum speed
    int rightspeed = 255;
    if (twist.linear.x > 1.0) {
        forward (leftspeed,rightspeed);
    } else if (twist.linear.x < -1.0) {
        reverse (leftspeed,rightspeed);
    } else if (twist.angular.z > 1.0) {
        left (leftspeed,rightspeed);
    } else if (twist.angular.z < -1.0) {
        right (leftspeed,rightspeed);
    } else {
        stop();
    }
}

void setup()
{
  // init
  nh.initNode();
  // advertise
  nh.advertise(chatter);
  // subscribe
  nh.subscribe(velocity_sub);
  setupMotor();
}

void loop()
{
    // interval
    unsigned long currentMillis = millis();
    // 1
    if (currentMillis - previousMillis1 >= interval1) {
      previousMillis1 = currentMillis;
      intervalStarted1 = true;
      numIntervals1++;
    }
    // 2
    if (currentMillis - previousMillis2 >= interval2) {
      previousMillis2 = currentMillis;
      intervalStarted2 = true;
      numIntervals2++;
    }
  if (intervalStarted2) {
      str_msg.data = hello;
      chatter.publish( &str_msg );
  }
  nh.spinOnce();
  loopMotor();
  intervalStarted1 = false;
  intervalStarted2 = false;
  delay(1);
}

