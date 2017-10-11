#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <rosserial_arduino/Adc.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

// will store last time
unsigned long previousMillis = 0;
// interval1 to do something in milliseconds
const long interval = 300;
int numIntervals = 0;
bool intervalStarted;

//------------------------- motors---------------------------------------------------
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
//int sensorMin1 = 1000; // only for debug
//int sensorMin2 = 1000;// only for debug
//int sensorMax1 = 0;// only for debug
//int sensorMax2 = 0;// only for debug
int sensorSwitch1 = 700; // initial guess for sensor switch
int sensorSwitch2 = 800; // initial guess for sensor switch
int sensorcount10 = 0;
int sensorcount11 = 0;
long count1 = 0; // left count of sensor state changes
long count1old = 0; // left count of sensor state changes, previous time step
int sensorcount20 = 0;
int sensorcount21 = 0;
long count2 = 0; // left count of sensor state changes
long count2old = 0; // left count of sensor state changes, previous time step
unsigned long previousMillisVelocity = 0;

//----------------------  velocity ----------------------
// velocity
geometry_msgs::Twist twist_msg_tracks; // raw track velocity, for debug only
geometry_msgs::Twist twist_msg_velocity; // estimated robot velocities, linear & angular


// velocity sensors
// if everything is published at the same time arduino will run out of memery
//ros::Publisher pub_velocityTracks("robby_track_1/velocityTracks", &twist_msg_tracks); // for debug only
ros::Publisher pub_velocity("robby_track_1/velocity", &twist_msg_velocity);

float velocityLeft = 0.0; // angular velocity of left track [rpm]
float velocityRight = 0.0; // angular velocity or right track [rpm]
float wheelDiameter = 0.036; // wheel diameter in [m]
float trackDistance = 0.09; // track distance in [m]

//----------------------  Servos ----------------------
Servo servoHor;  // create servo object to control a servo
Servo servoVert;  // create servo object to control a servo
const int servoHorPin   = 9;
const int servoVertPin  = 3;
int servoHorPos   = 78;
int servoVertPos  = 95;


//----------------------  sonar ----------------------
//const int sonarPin = 12; // pin connection

//sensor_msgs::Range range_msg_sonar; // message, distance from sonar_sensor
//ros::Publisher pub_range("robby_track_1/range", &range_msg_sonar);


// ++++++++++++++++++++++++ functions start +++++++++++++++++++++++++

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
// veloctiy callback
void velocityCallback(const geometry_msgs::Twist& vel)
{
    float vl; // left track velocity [m/s]
    float vr; // right track velocity [m/s]

    vl = vel.linear.x - trackDistance / 2.0 * vel.angular.z; // rough estimation only valid for small angles
    vr = 2.0 * vel.linear.x - vl; // rough estimation only valid for small angles

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

// subscriber for velocity
ros::Subscriber<geometry_msgs::Twist> velocity_sub("robby_track_1/cmd_vel", &velocityCallback);


void setupMotor()
{
  int i;
  for(i=E1;i<=M1;i++)
  pinMode(i, OUTPUT);
}



// callback for servo position, multiple subscribesr didnt work yet...
void servoCallback(const std_msgs::Int32& cmd_msg)
{
    servoHorPos = cmd_msg.data/1000; // first three digits are horizontal servo angle
    servoVertPos = cmd_msg.data - (cmd_msg.data/1000) * 1000;// last three digits are vertical servo angle

    servoHor.write(servoHorPos);
    servoVert.write(servoVertPos);
}

// subcriber for servo position
ros::Subscriber<std_msgs::Int32> subscriberServo("robby_track_1/servo", servoCallback);



void setupSensor() {
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
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

  // calculation of velocitie
  if (intervalStarted) {
      // time
      unsigned long currentMillisVelocity = millis();
      // velocity of tracks
      unsigned long timeDelta = currentMillisVelocity-previousMillisVelocity; // [millis s]
      velocityLeft = leftTrackDirection * (count1-count1old)/20.0 / (timeDelta / 1000.0); // rps
      velocityRight = rightTrackDirection * (count2-count2old)/20.0 / (timeDelta / 1000.0); // rps
      twist_msg_tracks.linear.x = velocityLeft  * wheelDiameter * 3.14159265359 ;// [m/s]
      twist_msg_tracks.linear.y = velocityRight  * wheelDiameter * 3.14159265359 ;// [m/s]
      twist_msg_tracks.angular.x = velocityLeft; // [rps]
      twist_msg_tracks.angular.y = velocityRight; // [rps]
      // velocity of robot
      //      dl = twist_msg_tracks.linear.x * (timeDelta / 1000.0); // left track travelled [m]
      //      dr = twist_msg_tracks.linear.y * (timeDelta / 1000.0); // right track travelled [m]
      dl = velocityLeft  * wheelDiameter * 3.14159265359 * (timeDelta / 1000.0); // left track travelled [m]
      dr = velocityRight  * wheelDiameter * 3.14159265359 * (timeDelta / 1000.0); // right track travelled [m]
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
      pub_velocity.publish(&twist_msg_velocity);
      

  }
  // publish for debug
  //twist_msg_tracks.linear.z = rawsensorValue1 ;// []
  //twist_msg_tracks.angular.z = rawsensorValue2; // []
  //pub_velocityTracks.publish(&twist_msg_tracks);
}

// +++++++++++++++++++++++++ SERVOS +++++++++++++++++++++++++++++++++++++++++++
void setupServo() {
    servoHor.attach(servoHorPin);  // attaches the servo on pin 9 to the servo object
    servoVert.attach(servoVertPin);  // attaches the servo on pin 9 to the servo object
    servoHor.write(servoHorPos);                  // sets the servo position according to the scaled value
    servoVert.write(servoVertPos);                  // sets the servo position according to the scaled value
}



//++++++++++++++++++ SONAR +++++++++++++++++++++++++
// removed to make space for servo subscribers, maybe change to leaner datatype and publish proper data type on kanga


//long microsecondsToCentimeters(long microseconds)
//{
//  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
//  // The ping travels out and back, so to find the distance of the
//  // object we take half of the distance travelled.
//  return microseconds / 29 / 2;
//}

//void loopUltra()
//{
//  // establish variables for duration of the ping,
//  // and the distance result in inches and centimeters:
//  long duration, inches, cm;

//  if (intervalStarted) {
//    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
//    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
//    pinMode(sonarPin, OUTPUT);
//    digitalWrite(sonarPin, LOW);
//    delayMicroseconds(2);
//    digitalWrite(sonarPin, HIGH);
//    delayMicroseconds(15);
//    digitalWrite(sonarPin, LOW);
//    delayMicroseconds(20);
//    // The same pin is used to read the signal from the PING))): a HIGH
//    // pulse whose duration is the time (in microseconds) from the sending
//    // of the ping to the reception of its echo off of an object.
//    pinMode(sonarPin, INPUT);
//    duration = pulseIn(sonarPin, HIGH);

//    // convert the time into a distance
//    cm = microsecondsToCentimeters(duration);

//    // put data into message
//    range_msg_sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
//    range_msg_sonar.header.frame_id =  "/ultraSonic"; // frame
//    range_msg_sonar.header.stamp = nh.now(); // time
//    range_msg_sonar.field_of_view = 60.0 / 360 * 3.14; // [rad]
//    range_msg_sonar.min_range = 0.03; // [m]
//    range_msg_sonar.max_range = 4.0; // [m]
//    range_msg_sonar.range = cm / 100.0; // [m]
//    // publish only if in detection window: how do i do that???
////    if ((range_msg_sonar.range >= range_msg_sonar.min_range)
////            &&
////            (range_msg_sonar.range <= range_msg_sonar.max_range)) {
//        pub_range.publish(&range_msg_sonar);

////    }
//  }
//}


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
  //nh.advertise(pub_velocityTracks);
  nh.advertise(pub_velocity);
//  nh.advertise(pub_range);
  // subscribe
  nh.subscribe(subscriberServo);
  nh.subscribe(velocity_sub);
}

void loop()
{
    // interval
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      intervalStarted = true;
      numIntervals++;
    }

  loopSensor();
 // loopUltra();

  // end
  intervalStarted = false;
  nh.spinOnce();
  delay(1);
}
