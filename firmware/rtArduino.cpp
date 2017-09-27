#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <roscpp/Logger.h>
#include <tf/tf.h>
#include <string.h>
#include <rosserial_arduino/Adc.h>

//#include <PID_v1.h>




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
int S1 = 0; // left sensor
int S2 = 1; // right sensor
int leftTrackDirection; // direction of left track, 1=forward, -1=backward)
int rightTrackDirection; // direction of left track, 1=forward, -1=backward)
int rawsensorValue1 = 0; // variable to store the value coming from the sensor
int rawsensorValue2 = 0; // variable to store the value coming from the sensor
int sensorMin1 = 1000;
int sensorMin2 = 1000;
int sensorMax1 = 0;
int sensorMax2 = 0;
int sensorSwitch1 = 700; // initial guess for sensor switch
int sensorSwitch2 = 790; // initial guess for sensor switch
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
float wheelDiameter = 0.036; // wheel diameter in [m]
float trackDistance = 0.09; // track distance in [m]
// PID
//double Setpoint, Input, Output;

//--- Servos ---
Servo servoHor;  // create servo object to control a servo
Servo servoVert;  // create servo object to control a servo
const int servoHorPin   = 9;
const int servoVertPin  = 3;
int servoHorPos   = 78;
int servoVertPos  = 100;

//geometry_msgs::Vector3 cmd_servo; // servo position


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

/*
// callback for servo position, multiple subscribesr didnt work yet...
void servoCallback(const std_msgs::UInt16& cmd_msg)
{
    //cmd_servo = cmd_msg;
    //servoHorPos = cmd_msg.z;
    //servoVertPos = cmd_msg.y;

//    servoHor.write(cmd_msg.data);
//    servoVert.write(cmd_msg.data);
}

// subcriber for servo position
ros::Subscriber<std_msgs::UInt16> subscriberServo("robby_track_1/servo", servoCallback);
*/
// service function translates velocity [m/s] into command for motor
int voltageFromVelocity (float velocity)
{
    // 100 is minimum speed (ca 0.01 m/s) 255 (ca 0.1 m/s) is maximum speed
    int minVoltage = 100; // for motors
    int maxVoltage = 255; // for motors
    float minVelocity = 0.01; // for tracks
    float maxVelocity = 0.1;  // for tracks

    int voltageRange;
    float velocityRange;
    float usedVelocity;

    voltageRange = maxVoltage - minVoltage;
    velocityRange = maxVelocity - minVelocity;

    if (abs(velocity) < minVelocity) {
        return 0;
    } else {
        usedVelocity = min(maxVelocity,abs(velocity));
        return minVoltage + (usedVelocity - minVelocity) * voltageRange / velocityRange;
    }

}

void loopMotor()
{

    float vl; // left track velocity [m/s]
    float vr; // right track velocity [m/s]

    vl = cmd_vel.linear.x - trackDistance / 2.0 * cmd_vel.angular.z; // rough estimation only valid for small angles
    vr = 2.0 * cmd_vel.linear.x - vl; // rough estimation only valid for small angles

    // left motor actuation
    analogWrite(E1, voltageFromVelocity(vl));
    // right motor actuation
    analogWrite(E2, voltageFromVelocity(vr));
    // leftTrackDirection
    if (vl < 0.0) {
        leftTrackDirection = -1;
        digitalWrite(M1, HIGH);
    } else {
        leftTrackDirection = 1;
        digitalWrite(M1, LOW);
    }
    // rightTrackDirection
    if (vr < 0.0) {
        rightTrackDirection = -1;
        digitalWrite(M2, HIGH);
    } else {
        rightTrackDirection = 1;
        digitalWrite(M2, LOW);
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
  float dl; // change of distance of left track [m]
  float dr; // change of distance of left track [m]
  float dx; // change of distance of robot in x-direction [m] (local reference frame)
  float dy; // change of distance of robot in y-direction [m] (local reference frame)
  float ds; // change of distance of robot in total [m]
  float dtheta; // change of angle of robot in z-axis [rad] (local reference frame)

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
      velocityLeft = leftTrackDirection * (count1-count1old)/20.0 / (timeDelta / 1000.0); // rps
      velocityRight = rightTrackDirection * (count2-count2old)/20.0 / (timeDelta / 1000.0); // rps
      twist_msg_tracks.linear.x = velocityLeft  * wheelDiameter * 3.14159265359 ;// [m/s]
      twist_msg_tracks.linear.y = velocityRight  * wheelDiameter * 3.14159265359 ;// [m/s]
      twist_msg_tracks.linear.z = rawsensorValue1 ;// []
      twist_msg_tracks.angular.x = velocityLeft; // [rps]
      twist_msg_tracks.angular.y = velocityRight; // [rps]
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

// +++++++++++++++++++++++++ SERVOS +++++++++++++++++++++++++++++++++++++++++++
void setupServo() {
    servoHor.attach(servoHorPin);  // attaches the servo on pin 9 to the servo object
    servoVert.attach(servoVertPin);  // attaches the servo on pin 9 to the servo object
    servoHor.write(servoHorPos);                  // sets the servo position according to the scaled value
    servoVert.write(servoVertPos);                  // sets the servo position according to the scaled value
}

void loopServo()
{
/*    servoHor.write(cmd_servo.x);
    servoVert.write(cmd_servo.y);

  int servoStep=10;
  int servoMin=0;
  int servoMax=180;
  if (serialReceived) {
    switch(serialVal) // Perform an action depending on the command
    {
      case 'i'://Move Forward
      case 'I':
        servoVertPos -= servoStep;
        servoVertPos = max(servoMin,servoVertPos);
        servoVert.write(servoVertPos);                  // sets the servo position according to the scaled value
        break;
      case 'k'://Move Backwards
      case 'K':
        servoVertPos += servoStep;
        servoVertPos = min(servoMax,servoVertPos);
        servoVert.write(servoVertPos);                  // sets the servo position according to the scaled value
        break;
      case 'j'://Turn Left
      case 'J':
        servoHorPos += servoStep;
        servoHorPos = min(servoMax,servoHorPos);
        servoHor.write(servoHorPos);                  // sets the servo position according to the scaled value
        break;
      case 'l'://Turn Right
      case 'L':
        servoHorPos -= servoStep;
        servoHorPos = max(servoMin,servoHorPos);
        servoHor.write(servoHorPos);                  // sets the servo position according to the scaled value
        break;
      default:
        break;
    }
  }
  */
}


// +++++++++++++++++++++++++ main loop +++++++++++++++++++++++++++++++++++++++++++
void setup()
{
  // init
  nh.initNode();
  // setup
  setupMotor();
  setupSensor();
  setupServo();
  // advertise
  nh.advertise(pub_velocitySensorRaw);
  nh.advertise(pub_velocityTracks);
  nh.advertise(pub_velocity);
  // subscribe
  //nh.subscribe(subscriberServo);
  nh.subscribe(velocity_sub);
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
  loopServo();
  // interval 2
  if (intervalStarted2) {
  }
  nh.spinOnce();
  // end
  intervalStarted1 = false;
  intervalStarted2 = false;
  delay(1);
}
