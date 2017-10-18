#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <rosserial_arduino/Adc.h>
#include <sensor_msgs/Range.h>
#include <robby_track2/pwmDirectional2.h>

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
robby_track2::pwmDirectional2 pwm_msg; // incoming pwm control of motors

//------- rotational sensors ----------
int S1 = 0; // left sensor
int S2 = 1; // right sensor
int leftTrackDirection; // direction of left track, 1=forward, -1=backward)
int rightTrackDirection; // direction of left track, 1=forward, -1=backward)
int rawsensorValue1 = 0; // variable to store the value coming from the sensor
int rawsensorValue2 = 0; // variable to store the value coming from the sensor
// sensor messages
robby_track2::pwmDirectional2 sensor_msg; // outgoing sensor signal
ros::Publisher pub_sensor_tracks("robby_track_1/sensor_tracks", &sensor_msg);

//----------------------  Servos ----------------------
Servo servoHor;  // create servo object to control a servo
Servo servoVert;  // create servo object to control a servo
const int servoHorPin   = 9;
const int servoVertPin  = 3;
int servoHorPos   = 74;
int servoVertPos  = 95;


//----------------------  sonar ----------------------
const int sonarPin = 12; // pin connection

sensor_msgs::Range range_msg_sonar; // message, distance from sonar_sensor
ros::Publisher pub_range("robby_track_1/range", &range_msg_sonar);

// ++++++++++++++++++++++++ motor +++++++++++++++++++++++++
// veloctiy callback
void pwmCallback(const robby_track2::pwmDirectional2& msg)
{
  // left motor actuation
  analogWrite(E1, abs(msg.pwmDirection1));
  // right motor actuation
  analogWrite(E2, abs(msg.pwmDirection2));
  // leftTrackDirection, no change when zero
  if (msg.pwmDirection1 < 0) {
    leftTrackDirection = -1;
    digitalWrite(M1, HIGH);
  } else if (msg.pwmDirection1 > 0){
    leftTrackDirection = 1;
    digitalWrite(M1, LOW);
  }
  // rightTrackDirection
  if (msg.pwmDirection2 < 0) {
    rightTrackDirection = -1;
    digitalWrite(M2, HIGH);
  } else if (msg.pwmDirection2 > 0){
    rightTrackDirection = 1;
    digitalWrite(M2, LOW);
  }
}

// subscriber for motor control
ros::Subscriber<robby_track2::pwmDirectional2> motor_sub("robby_track_1/motor_pwm", &pwmCallback);

void setupMotor()
{
  int i;
  for(i=E1;i<=M1;i++)
  pinMode(i, OUTPUT);
}


// ++++++++++++++++++++++++ sensor for motor +++++++++++++++++++++++++
void setupSensor() {
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
}

void loopSensor() {
  String output;
  String outputHeader;
  rawsensorValue1 = analogRead(S1);
  rawsensorValue2 = analogRead(S2);
 
  // sensor data
  sensor_msg.pwmDirection1 = leftTrackDirection * rawsensorValue1 ;// []
  sensor_msg.pwmDirection2 = rightTrackDirection * rawsensorValue2; // []
  pub_sensor_tracks.publish(&sensor_msg);
}

// +++++++++++++++++++++++++ SERVOS +++++++++++++++++++++++++++++++++++++++++++

// callback for servo position, multiple subscribesr didnt work yet...
void servoCallback(const robby_track2::pwmDirectional2& cmd_msg)
{
    servoHorPos = cmd_msg.pwmDirection1;
    servoVertPos = cmd_msg.pwmDirection2;

    servoHor.write(servoHorPos);
    servoVert.write(servoVertPos);
}
// subcriber for servo position
ros::Subscriber<robby_track2::pwmDirectional2> subscriberServo("robby_track_1/servo_corr", servoCallback);

// setup
void setupServo() {
    servoHor.attach(servoHorPin);  // attaches the servo on pin 9 to the servo object
    servoVert.attach(servoVertPin);  // attaches the servo on pin 9 to the servo object
    servoHor.write(servoHorPos);                  // sets the servo position according to the scaled value
    servoVert.write(servoVertPos);                  // sets the servo position according to the scaled value
}

//++++++++++++++++++ SONAR +++++++++++++++++++++++++
// removed to make space for servo subscribers, maybe change to leaner datatype and publish proper data type on kanga
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void loopUltra()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  if (intervalStarted) {
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(sonarPin, OUTPUT);
    digitalWrite(sonarPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sonarPin, HIGH);
    delayMicroseconds(15);
    digitalWrite(sonarPin, LOW);
    delayMicroseconds(20);
    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(sonarPin, INPUT);
    duration = pulseIn(sonarPin, HIGH);

    // convert the time into a distance
    cm = microsecondsToCentimeters(duration);

    // put data into message
    range_msg_sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg_sonar.header.frame_id =  "/ultraSonic"; // frame
    range_msg_sonar.header.stamp = nh.now(); // time
    range_msg_sonar.field_of_view = 60.0 / 360 * 3.14; // [rad]
    range_msg_sonar.min_range = 0.03; // [m]
    range_msg_sonar.max_range = 4.0; // [m]
    range_msg_sonar.range = cm / 100.0; // [m]
    // publish
    pub_range.publish(&range_msg_sonar);
  }
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
  nh.advertise(pub_sensor_tracks);
  nh.advertise(pub_range);
  // subscribe
  nh.subscribe(subscriberServo);
  nh.subscribe(motor_sub);
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

  // loop
  loopSensor();
  loopUltra();

  // end
  intervalStarted = false;
  nh.spinOnce();
  // not too fast, otherwise buffer overflow
  delay(10);
}
