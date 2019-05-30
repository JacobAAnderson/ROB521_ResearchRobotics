
// rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0

#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
geometry_msgs::Point target;
std_msgs::String str_msg;




void targetCallBack(const geometry_msgs::Point& msg)
{
  target = msg;
}

ros::Subscriber <geometry_msgs::Point> sub("/target", targetCallBack);
ros::Publisher chatter("arduino_debug", &str_msg);

char hello[50];

void setup()
{
  target.x = 0.0;
  target.y = 0.0;
  target.z = 0.0;
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
} 

void loop()
{

 float xx = target.x;
 float yy = target.y; 
 float zz = target.z;  

 char buff1[10];
 dtostrf(xx, 2,2, buff1);

 char buff2[10];
 dtostrf(yy, 2,2, buff2);

 char buff3[10];
 dtostrf(zz, 2,2, buff3);
 
 sprintf(hello,"Target: %s, %s, %s", buff1, buff2, buff3);
 str_msg.data = hello; // buff1;
 chatter.publish( &str_msg );
 
 nh.spinOnce();

 delay(1000);
  
}
