#include <ros.h>
#include <geometry_msgs/Twist.h>
// ROS
ros::NodeHandle  nodeHandle;
geometry_msgs::Twist TwistMsg;
ros::Publisher cmd_vel_pub("/cmd_vel", &TwistMsg); 

 int ledPin = 13;
 int joyPin1 = 0;                 // slider variable connecetd to analog pin 0
 int joyPin2 = 1;                 // slider variable connecetd to analog pin 1
 int value1 = 0;                  // variable to read the value from the analog pin 0
 int value2 = 0;                  // variable to read the value from the analog pin 1

 void setup() {
  Serial.begin(57600);
  nodeHandle.initNode();
  nodeHandle.advertise(cmd_vel_pub);  // This can be useful for debugging purposes

  pinMode(ledPin, OUTPUT);              // initializes digital pins 0 to 7 as outputs
  TwistMsg.angular.z = 0.0;
  TwistMsg.linear.x = 0.0;
  TwistMsg.linear.y = 0.0;
  TwistMsg.linear.z = 0.0;

  delay(1000) ;
 }

 void loop() {
  // reads the value of the variable resistor 
  value1 = analogRead(joyPin1);   
  // this small pause is needed between reading
  // analog pins, otherwise we get the same value twice
  // reads the value of the variable resistor 
  value2 = analogRead(joyPin2);   
  TwistMsg.angular.z = (float)(value2-500.0)/5000.0;
  TwistMsg.linear.x = (float)(value1-465.0)/-1000.0;
  TwistMsg.linear.y = 0.0;
  TwistMsg.linear.z = 0.0;

  cmd_vel_pub.publish( &TwistMsg );
  nodeHandle.spinOnce();
  delay(100);             
 }
