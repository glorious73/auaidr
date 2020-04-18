/*
 * ROS Subscriber to geometry_msgs/twist
 * Moves the robot based on commands from KeyboardTeleop.js
 * Also, the robot moves from the joystick
 * ----------------
 * bmsPublisher for monitoring the battery (V5)
 * ----------------
 * percent publisher for battery percentage
 */

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <string.h>

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
// -------- bms Variables --------
std_msgs::String bms_msg;
ros::Publisher bmsPublisher("/bms", &bms_msg);
String bmsString;

int potPin   = 2;
int p3 = 3;
int p4 = 4;
int p5 = 5;
int potValue = 0; // Initially
int pv3 = 0;
int pv4 = 0;
int pv5 = 0;
float bmsArray[13] = {2.1, 3.3, 2.0, 1.9, 4.1, 3.5, 2.9, 2.6, 2.5, 1.8, 2.3, 2.3, 4.0}; // dummy initial array

// -------- percent --------
std_msgs::String prcnt_msg;
ros::Publisher percentPublisher("/percentage", &prcnt_msg);
String prcntString;
int percentPotPin = 7;
int percentPotValue = 0; // Initially
float percent = 0;

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
  
  // ---- bms ----
  nh.advertise(bmsPublisher);
  bmsString = String(""); // Initially
  // ---- /bms ----

  // ---- percent ----
  nh.advertise(percentPublisher);
  prcntString = String(""); // Initially
  // ---- /percent -----
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
    // bms
    pinMode(potPin, INPUT);
    pinMode(p3, INPUT);
    pinMode(p4, INPUT);
    pinMode(p5, INPUT);
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
  // ----- bms -----
  // Change one of the values by reading the potentiometer
  potValue = analogRead(potPin);
  pv3 = analogRead(p3);
  pv4 = analogRead(p4);
  pv5 = analogRead(p5);
  bmsArray[4] = map(potValue, 0, 1023, 1.7, 4.2);
  bmsArray[5] = map(pv3, 0, 1023, 1.7, 4.2);
  bmsArray[6] = map(pv4, 0, 1023, 1.7, 4.2);
  bmsArray[7] = map(pv5, 0, 1023, 1.7, 4.2);
  // Make the array of voltages a string and delimit by a comma
  for(int i=0; i < 13; i++) {
    // No comma after the last voltage value
    bmsString = bmsString + bmsArray[i];
    if(i != 12) {
      bmsString = bmsString + ",";
    }
  }
  // send the string delimeted by the comma
    // 1. convert to char[] array
  char bmsBuffer[bmsString.length()+1];
  bmsString.toCharArray(bmsBuffer, bmsString.length()+1);
    // 2. send the char[] array to match std_msgs::String data type   
  bms_msg.data = bmsBuffer;
  bmsPublisher.publish( &bms_msg );
  // reset the bmsString
  bmsString = String("");
  // ----- /bms -----


  // ---- percent ----
  // Change one of the values by reading the potentiometer
  percentPotValue = analogRead(percentPotPin);
  percent = map(percentPotValue, 0, 1023, 0, 100);
  // Make the percentage a string
  prcntString = prcntString + percent;
    
  
  // send the percent string
    // 1. convert to char[] array
  char prcntBuffer[prcntString.length()+1];
  prcntString.toCharArray(prcntBuffer, prcntString.length()+1);
    // 2. send the char[] array to match std_msgs::String data type   
  prcnt_msg.data = prcntBuffer;
  percentPublisher.publish( &prcnt_msg );
  // reset the string
  prcntString = String("");
  // ---- /percnt ----
  // spin nodeHandle to listen to all messages in the ROS system
  nh.spinOnce();
  delay(1);
}
