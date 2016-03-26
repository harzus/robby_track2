#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <roscpp/Logger.h>
#include <tf/tf.h>
#include <string.h>
//#include <sensor_msgs/ChannelFloat32.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float32.h>

#include <Arduino.h>

ros::NodeHandle nh;

char debug[]= "debug statements";

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
String outString;
ros::Publisher chatter("chatter", &str_msg);
const int msgLen = 3;
char charMsg[msgLen] = "12";

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
int sensorSwitch1 = 700;
int sensorSwitch2 = 750;
int sensorcount10 = 0;
int sensorcount11 = 0;
long count1 = 0;
int sensorcount20 = 0;
int sensorcount21 = 0;
long count2 = 0;

// velocity
geometry_msgs::Twist twist;
// velocity sensors
//sensor_msgs::ChannelFloat32 velocityRaw_msg;
//ros::Publisher pub_velocityRaw( "velocityRaw", &velocityRaw_msg);

std_msgs::Float32 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);

rosserial_arduino::Adc adc_msg;
ros::Publisher pub_velocityRaw("velocityRaw", &adc_msg);

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
  //Min value is 400 and max value is 800, so state chance can be done at 600.
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
  if (rawsensorValue2 < sensorSwitch2){ //Min value is 400 and max value is 800, so state chance can be done at 600.
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
  if (intervalStarted2) {
      adc_msg.adc0 = sensorMin1;
      adc_msg.adc1 = sensorMax1;
      adc_msg.adc2 = count1;
      adc_msg.adc3 = sensorMin2;
      adc_msg.adc4= sensorMax2;
      adc_msg.adc5 = count2;
      pub_velocityRaw.publish(&adc_msg);
  }
}


void setup()
{
  // init
  nh.initNode();
  // advertise
  nh.advertise(chatter);
  nh.advertise(pub_sonar);
  nh.advertise(pub_velocityRaw);
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
    str_msg.data =  charMsg;
    chatter.publish(&str_msg);
    float sensorReading=2.2;
    sonar_msg.data = sensorReading;
    pub_sonar.publish(&sonar_msg);
     //      nh.logdebug(charMsg);
  }
  nh.spinOnce();
  // end
  intervalStarted1 = false;
  intervalStarted2 = false;
  delay(1);
}
