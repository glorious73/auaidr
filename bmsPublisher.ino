/*
 * rosserial BMS Publisher
 * Publishes 13 values for battery voltages in a battery pack
 * Developed by: Amjad Abujamous
 */
// ---- ----
#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

ros::NodeHandle  nh;


std_msgs::String bms_msg;
ros::Publisher bmsPublisher("/bms", &bms_msg);
String bmsString;

int potPin         = 2;
int p3 = 3;
int p4 = 4;
int p5 = 5;
int potValue       = 0; // Initially
int pv3 = 0;
int pv4 = 0;
int pv5 = 0;
float bmsArray[13] = {2.1, 3.3, 2.0, 1.9, 4.1, 3.5, 2.9, 2.6, 2.5, 1.8, 2.3, 2.3, 4.0}; // dummy initial array

void setup()
{
  nh.initNode();
  nh.advertise(bmsPublisher);
  bmsString = String(""); // Initially
}

void loop()
{
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
  nh.spinOnce();
  delay(100);
}
