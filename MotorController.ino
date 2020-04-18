/*
 * ROS Subscriber to geometry_msgs/twist
 * Moves the robot based on commands from KeyboardTeleop.js
 * Also, the robot moves from the joystick
 */

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// -------- Variables --------
ros::NodeHandle  nh;
// MOTOR Variables:
byte M1P = 2;
byte M1N = 3;
byte M2P = 4;
byte M2N = 5;
byte M3P = 6;
byte M3N = 7;
byte M4P = 8;
byte M4N = 9;
// Received (mapped 0-255) messages
int linear  = 0; // initially
int angular = 0; // initially
// Duty ratio variables
byte d1 = 0;
byte d2 = 0;
byte d3 = 0;
byte d4 = 0;
// Clearance
byte angularClearance = 10;
// -------- Message Callbacks --------
void dutyCallback(const std_msgs::Int16MultiArray& msg){
   // Does the callback work? Yes (checked)
   // Set dutyRatios to the ones received as well as threshold (working)
   linear    = msg.data[0];
   angular   = msg.data[1];
   /* Subscribing to the duty_ratios message and moving the motors with those values after determining whether positive or negative */
   // Linear (Y-axis)
   d1, d2 = linear; // both motors either move forward or backward (the difference between them rotates the robot)
   // Angular (X-axis)
   if(angular > angularClearance) {
    // move right --> increase speed of left motor and decrease right
    d1 = d1 - angular;
    d2 = d2 + angular;
   } else if(angular < -angularClearance) {
    // move left --> increase speed of right motor and decrease left
    d1 = d1 + angular;
    d2 = d2 - angular;
   } else {
    d2 = d1; // same same
   }
   // Set d3 equal to d1 and d4 equal to d2 (same side)
   d3 = d1;
   d4 = d2;
   // Write to motors
   if(linear > 0) {
    // forward
     analogWrite(M1P, d1);
     analogWrite(M3P, d3);
     analogWrite(M2P, d2);
     analogWrite(M4P, d4);
   } else {
    // backward
     analogWrite(M1N, d1);
     analogWrite(M3N, d3);
     analogWrite(M2N, d2);
     analogWrite(M4N, d4);
   }
   // Make the robot move for 0.5 s
   delay(500);
   // FINALLY, turn all switches off to prevent current overshoot
   analogWrite(M1P, 0);
   analogWrite(M1N, 0);
   analogWrite(M2P, 0);
   analogWrite(M2N, 0);
   analogWrite(M3P, 0);
   analogWrite(M3N, 0);
   analogWrite(M4P, 0);
   analogWrite(M4N, 0);
   delay(5); // safety
}

// -------- Subscriber objects --------; 
ros::Subscriber<std_msgs::Int16MultiArray> sub_duty("/duty_ratios", &dutyCallback);  

void setup() {
  // initNode and subscribe to all topics
  nh.initNode();
  nh.subscribe(sub_duty);
  // pinModes
    // MOTOR
    pinMode(M1P, OUTPUT);
    pinMode(M1N, OUTPUT);
    pinMode(M2P, OUTPUT);
    pinMode(M2N, OUTPUT);
    pinMode(M3P, OUTPUT);
    pinMode(M3N, OUTPUT);
    pinMode(M4P, OUTPUT);
    pinMode(M4N, OUTPUT);
  // Initialize all pins to 0
    analogWrite(M1P, 0);
    analogWrite(M1N, 0);
    analogWrite(M2P, 0);
    analogWrite(M2N, 0);
    analogWrite(M3P, 0);
    analogWrite(M3N, 0);
    analogWrite(M4P, 0);
    analogWrite(M4N, 0);
}

void loop() {
  // spin nodeHandle to listen to all messages in the ROS system
  nh.spinOnce();
  delay(1);
}
