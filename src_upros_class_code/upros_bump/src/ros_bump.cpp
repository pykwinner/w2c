#include "ros/ros.h"  
#include "std_msgs/Int16MultiArray.h"  
 
void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)  
{  
  ROS_INFO("Bump Sensor Data Received: ");  
  for (int i = 0; i < msg->data.size(); ++i)  
  {  
    ROS_INFO("Sensor %d: %s", i, msg->data[i] ? "Triggered" : "Not Triggered");  
  }  
}  
 
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "bump_sensor_subscriber");  
  ros::NodeHandle n;  
 
  ros::Subscriber sub = n.subscribe("/robot/bump_sensor", 1000, bumpCallback);  
 
  ros::spin();  
 
  return 0;  
}