/*
 * rosserial percent Publisher
 * Publishes percentage for battery voltage
 * Developed by: Amjad Abujamous
 */
// ---- ----
#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

ros::NodeHandle  nh;


std_msgs::String prcnt_msg;
ros::Publisher percentPublisher("/percentage", &prcnt_msg);
String prcntString;

int potPin    = 2;
int potValue  = 0; // Initially
float percent = 0; // Initially

void setup()
{
  nh.initNode();
  nh.advertise(percentPublisher);
  prcntString = String(""); // Initially
}

void loop()
{
  // Change one of the values by reading the potentiometer
  potValue = analogRead(potPin);
  percent = map(potValue, 0, 1023, 0, 100);
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
  nh.spinOnce();
  delay(100);
}
