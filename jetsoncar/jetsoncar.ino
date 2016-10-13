/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/
/*

  Plat4. Inc, (Yukihiro SAITO)
  email : yukihiro.saito@tier4.jp
  MIT License
*/
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <MsTimer2.h>
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

// Wcheel Encoder
#define ECA 2   // D2   
#define ECB 3   // D3 
#define inputEc(x) digitalRead(x)   //  
byte curDat;  
byte befDat = 0;  
byte rotDir = 0;  
int Count = 0;  
byte inputMatch;  
byte matchCnt;    
byte rotPat = 0;  

// ROS
ros::NodeHandle  nodeHandle;

// TRAXXAS Electronic Speed Controller (ESC)
const int minSteering = 30 ;
const int maxSteering = 150 ;
const int minThrottle = 0 ;
const int maxThrottle = 150 ;
const float steeringScale = 0.3;
const float steeringOffset = 0.5;
const float electronicSpeedScale = 0.1;
const float electronicSpeedOffset = 0.51;
Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo


// For DEBUG
std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 


std_msgs::Int32 wheel_encode_count_msg;
ros::Publisher wheel_encode_counter("wheel_encode_count", &wheel_encode_count_msg); 
geometry_msgs::Twist targetTwistMsg;

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

signed char checkEnc(byte dat) {  
  byte pat,i,j;  
  
  rotPat <<= 2;  
  rotPat |= (dat & 0x03);  
  
  if(rotPat == 0x4B) {        // +パターン  
    return 1;  
  } else if(rotPat == 0x87) {     // -パターン  
    return -1;  
  } else {  
    return 0;  
  }  
}  

void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{
  targetTwistMsg = twistMsg;
  
  int steeringAngle = fmap(-1.0 * (twistMsg.angular.z * steeringScale) + steeringOffset, 0.0, 1.0, minSteering, maxSteering) ;
  // The following could be useful for debugging
  // str_msg.data= steeringAngle ;
  // chatter.publish(&str_msg);
  // Check to make sure steeringAngle is within car range
  if (steeringAngle < minSteering) { 
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    steeringAngle = maxSteering ;
  }
  steeringServo.write(steeringAngle) ;
  
  // ESC forward is between 0.5 and 1.0
  int escCommand ;
  if (twistMsg.linear.x >= 0.5) {
    escCommand = (int)fmap(twistMsg.linear.x * electronicSpeedScale + electronicSpeedOffset, 0.5, 1.0, 90.0, maxThrottle) ;
  } else {
    escCommand = (int)fmap(twistMsg.linear.x * electronicSpeedScale + electronicSpeedOffset, 0.0, 1.0, 0.0, 180.0) ;
  }
  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) { 
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }
  // The following could be useful for debugging
  // str_msg.data= escCommand ;
  // chatter.publish(&str_msg);
  
  electronicSpeedController.write(escCommand) ;
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
 
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;

void wec_publish () {
   wheel_encode_count_msg.data = Count;
   wheel_encode_counter.publish( &wheel_encode_count_msg );
   Count = 0;
}

void setup(){
  pinMode(13, OUTPUT);
  Serial.begin(57600) ;
  
  // ROS
  nodeHandle.initNode();
  nodeHandle.advertise(chatter);  // This can be useful for debugging purposes
  nodeHandle.advertise(wheel_encode_counter);  // This can be useful for debugging purposes
  nodeHandle.subscribe(driveSubscriber) ;  // Subscribe to the steering and throttle messages

  // Wheel Encoder
  pinMode(ECA, INPUT);  
  pinMode(ECB, INPUT);  
  curDat = 0;  
  if(inputEc(ECA)) {  
    befDat |= 2;  
  }  
  if(inputEc(ECB)) {  
    befDat |= 1;  
  }
  MsTimer2::set(200, wec_publish); // publish wheel encode count
  MsTimer2::start();
  
  // TRAXXAS Electronic Speed Controller (ESC)
  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(10); // ESC is on pin 10
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steeringServo.write(90) ;
  electronicSpeedController.write(90) ;
  
  // Controll
  targetTwistMsg.angular.z = 0.0;
  targetTwistMsg.liner.x = 0.0;
  targetTwistMsg.liner.y = 0.0;
  targetTwistMsg.liner.z = 0.0;
  
  delay(1000) ;
  
}

void loop(){
  nodeHandle.spinOnce();
  byte dat;  
  signed char val;  
  
  curDat = 0;  
  if(inputEc(ECA)) {  
    curDat |= 2;  
  }  
  if(inputEc(ECB)) {  
    curDat |= 1;  
  }  
  
  if(befDat == curDat) {  
    if(!inputMatch) {           //  既に一致しているときは 何もしない  
      matchCnt++;  
      if(matchCnt >= 5) {  
        // 状態確定  
        inputMatch = true;  
        val = checkEnc(curDat);  
        if(val != 0) {  
          Count += val;  
          Serial.println(Count);  
        }  
      }  
    }  
  } else {  
    befDat = curDat;  
    matchCnt = 0;  
    inputMatch = false;  
  }  
  
//  delay(1);
}
