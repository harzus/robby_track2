#include <Arduino.h>
#include <ros.h>

//Contains the bridge code between the API and the Arduino Environment
#include "arduino_library_nine_axes_motion/NineAxesMotion.h"        
#include <Wire.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

NineAxesMotion mySensor;         //Object that for the sensor 
unsigned long lastStreamTime = 0; //To store the last streamed time stamp
const int streamPeriod = 20;  //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))

// sensor messages
geometry_msgs::Quaternion quat_msg; // outgoing sensor signal 
ros::Publisher pub_sensor_quat("robby_track_1/quaternion", &quat_msg);
geometry_msgs::Twist twist_msg; // outgoing sensor signal 
ros::Publisher pub_sensor_twist("robby_track_1/twist", &twist_msg);


// +++++++++++++++++++++++++ main loop +++++++++++++++++++++++++++++++++++++++++++
void setup()
{
  // init
  nh.initNode();
  I2C.begin(); 
  // setup
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  nh.advertise(pub_sensor_quat);
  nh.advertise(pub_sensor_twist);
}


void loop() //This code is looped forever
{

  //if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();
	  mySensor.updateQuat();
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
		mySensor.updateGyro(); // update gyro
    mySensor.updateCalibStatus();  //Update the Calibration Status

    quat_msg.x = mySensor.readQuatX();
    quat_msg.y = mySensor.readQuatX();
    quat_msg.z = mySensor.readQuatY();
    quat_msg.w = mySensor.readQuatW();

    twist_msg.linear.x = mySensor.readLinearAccelX();	
    twist_msg.linear.y = mySensor.readLinearAccelY();	
    twist_msg.linear.z = mySensor.readLinearAccelZ();	
    twist_msg.angular.x = mySensor.readGyroX();	
    twist_msg.angular.y = mySensor.readGyroY();	
    twist_msg.angular.z = mySensor.readGyroZ();	

    pub_sensor_quat.publish(&quat_msg);
    pub_sensor_twist.publish(&twist_msg);

  }
  nh.spinOnce();
  delay(10);
}
