#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <roscpp/Logger.h>
#include <tf/tf.h>
#include <string.h>
#include <rosserial_arduino/Adc.h>

//#include <PID_v1.h>

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

//To control the rover, Copy and paste the code below into the Arduino software
int E1 = 6; //M1 Speed Control
int E2 = 5; //M2 Speed Control
int M1 = 8; //M1 Direction Control
int M2 = 7; //M2 Direction Control

//------- rotational sensors ----------
// The code below simply counts the number of changes,
// so a disc with 8x white sections and 8x cutouts will
// provide a count of 16 per 360 degree rotation. It is
// up to you to integrate it with your code

int S1 = 0; // left sensor
int S2 = 1; // right sensor
int rawsensorValue1 = 0; // variable to store the value coming from the sensor
int rawsensorValue2 = 0; // variable to store the value coming from the sensor
int sensorMin1 = 1000;
int sensorMin2 = 1000;
int sensorMax1 = 0;
int sensorMax2 = 0;
int sensorSwitch1 = 700; // initial guess for sensor switch
int sensorSwitch2 = 790; // initial guess for sensor switch
int sensorSwitchAutomatic = 1e20; // after x sensor counts sensorSwitch1&2 will be the average of sensorMin & sensorMax
int sensorcount10 = 0;
int sensorcount11 = 0;
long count1 = 0; // left count of sensor state changes
long count1old = 0; // left count of sensor state changes, previous time step
int sensorcount20 = 0;
int sensorcount21 = 0;
long count2 = 0; // left count of sensor state changes
long count2old = 0; // left count of sensor state changes, previous time step
unsigned long previousMillisVelocity = 0;

// velocity
geometry_msgs::Twist cmd_vel; // target velocity, assumed to be in local reference frame (x: front, y:side, z: top)
geometry_msgs::Twist twist_msg_tracks; // raw track velocity
geometry_msgs::Twist twist_msg_velocity; // estimated robot velocities, linear & angular

// velocity sensors
rosserial_arduino::Adc adc_msg; // raw sensor data
ros::Publisher pub_velocitySensorRaw("robby_track_1/velocitySensorRaw", &adc_msg);
ros::Publisher pub_velocityTracks("robby_track_1/velocityTracks", &twist_msg_tracks);
ros::Publisher pub_velocity("robby_track_1/velocity", &twist_msg_velocity);

float velocityLeft = 0.0; // angular velocity of left track [rpm]
float velocityRight = 0.0; // angular velocity or right track [rpm]
float wheelDiameter = 0.0036; // wheel diameter in [m]
float trackDistance = 0.09; // track distance in [m]
float dl = 0.0; // change of distance of left track [m]
float dr = 0.0; // change of distance of left track [m]
float dx = 0.0; // change of distance of robot in x-direction [m] (local reference frame)
float dy = 0.0; // change of distance of robot in y-direction [m] (local reference frame)
float ds = 0.0; // change of distance of robot in total [m]
float dtheta = 0.0; // change of angle of robot in z-axis [rad] (local reference frame)
// PID
//double Setpoint, Input, Output;


// veloctiy callback
void velocityCallback(const geometry_msgs::Twist& vel)
{
  cmd_vel = vel;
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
    int leftspeed = 150; //255 is maximum speed
    int rightspeed = 150;
    if (cmd_vel.linear.x > 1.0) {
        forward (leftspeed,rightspeed);
    } else if (cmd_vel.linear.x < -1.0) {
        reverse (leftspeed,rightspeed);
    } else if (cmd_vel.angular.z > 1.0) {
        left (leftspeed,rightspeed);
    } else if (cmd_vel.angular.z < -1.0) {
        right (leftspeed,rightspeed);
    } else {
        stop();
    }
}

void setupSensor() {
  int i;
  for(i=10;i<=12;i++)
    pinMode(i, OUTPUT);
}

void loopSensor() {
  String output;
  String outputHeader;
  rawsensorValue1 = analogRead(S1);
  rawsensorValue2 = analogRead(S2);

  // count rotations
  // left
  if (rawsensorValue1 < sensorSwitch1){
    sensorcount11 = 1;
  }
  else {
    sensorcount11 = 0;
  }
  if (sensorcount11 != sensorcount10){
    count1 ++;
  }
  sensorcount10 = sensorcount11;
  // right
  if (rawsensorValue2 < sensorSwitch2){
    sensorcount21 = 1;
  }
  else {
    sensorcount21 = 0;
  }
  if (sensorcount21 != sensorcount20){
    count2 ++;
  }
  sensorcount20 = sensorcount21;

  // min / max
  if (rawsensorValue1 > sensorMax1) {
    sensorMax1 = rawsensorValue1;
  }
  if (rawsensorValue2> sensorMax2) {
    sensorMax2 = rawsensorValue2;
  }
  if (rawsensorValue1 < sensorMin1) {
    sensorMin1 = rawsensorValue1;
  }
  if (rawsensorValue2 < sensorMin2) {
    sensorMin2 = rawsensorValue2;
  }
  // update of sensorswitch
  if (count1 > sensorSwitchAutomatic) {
      sensorSwitch1 = (sensorMax1 + sensorMin1) / 2;
  }
  if (count2 > sensorSwitchAutomatic) {
      sensorSwitch1 = (sensorMax2 + sensorMin2) / 2;
  }
  // data raw
  adc_msg.adc0 = sensorMax1 - sensorMin1;
  adc_msg.adc1 = rawsensorValue1;
  adc_msg.adc2 = count1;
  adc_msg.adc3 = sensorMax2 - sensorMin2;
  adc_msg.adc4 = rawsensorValue2;
  adc_msg.adc5 = count2;
  // data raw publish
  pub_velocitySensorRaw.publish(&adc_msg);


  // calculation of velocities
  if (intervalStarted2) {
      // time
      unsigned long currentMillisVelocity = millis();
      // velocity of tracks
      unsigned long timeDelta = currentMillisVelocity-previousMillisVelocity; // [millis s]
      velocityLeft = (count1-count1old)/16.0 / timeDelta * 1000.0 * 60.0;
      velocityRight = (count2-count2old)/16.0 / timeDelta * 1000.0 * 60.0;
      twist_msg_tracks.linear.x = velocityLeft / 60 * wheelDiameter * 3.14159265359 ;// [m/s]
      twist_msg_tracks.linear.y = velocityRight / 60 * wheelDiameter * 3.14159265359 ;// [m/s]
      twist_msg_tracks.linear.z = rawsensorValue1 ;// []
      twist_msg_tracks.angular.x = velocityLeft; // [rpm]
      twist_msg_tracks.angular.y = velocityRight; // [rpm]
      twist_msg_tracks.angular.z = rawsensorValue2; // []
      // velocity of robot
      dl = twist_msg_tracks.linear.x * (timeDelta / 1000.0); // left track travelled [m]
      dr = twist_msg_tracks.linear.y * (timeDelta / 1000.0); // right track travelled [m]
      ds = (dl + dr) / 2.0; // robot travelled [m]
      dtheta = (dr - dl) / trackDistance; // angular change [rad]
      dx = ds * cos(dtheta / 2.0); // change in x-direction [m]
      dy = ds * sin(dtheta / 2.0); // change in y-direction [m]
      // fill twist_msg_velocity
      twist_msg_velocity.linear.x = dx / (timeDelta / 1000.0); // [m/s]
      twist_msg_velocity.linear.y = dy / (timeDelta / 1000.0); // [m/s]
      twist_msg_velocity.linear.z = 0.0;
      twist_msg_velocity.angular.x = 0.0;
      twist_msg_velocity.angular.y = 0.0;
      twist_msg_velocity.angular.z = dtheta / (timeDelta / 1000.0); // [rad/s]
      // update
      previousMillisVelocity = currentMillisVelocity;
      count1old = count1;
      count2old = count2;
      // publish
      pub_velocityTracks.publish(&twist_msg_tracks);
      pub_velocity.publish(&twist_msg_velocity);
  }
}


void setup()
{
  // init
  nh.initNode();
  // advertise
  nh.advertise(pub_velocitySensorRaw);
  nh.advertise(pub_velocityTracks);
  nh.advertise(pub_velocity);
  // subscribe
  nh.subscribe(velocity_sub);
  setupMotor();
  setupSensor();
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
  loopMotor();
  loopSensor();
  // interval 2
  if (intervalStarted2) {
  }
  nh.spinOnce();
  // end
  intervalStarted1 = false;
  intervalStarted2 = false;
  delay(1);
}
