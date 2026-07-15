#include <ros/ros.h>

//包含必要的头文件，其中TutorialsConfig.h就是配置文件在编译过程中生成的头文件。
#include <dynamic_reconfigure/server.h>
#include <upros_message/TutorialsConfig.h>
#include <geometry_msgs/Twist.h>

double robot_speed = 0;

//定义回调函数，并将回调函数和服务端绑定。当客户端请求修改参数时，服务端即可跳转到回调函数进行处理
void callback(upros_message::TutorialsConfig &config, uint32_t level)
{
 ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
           config.int_param, config.double_param, 
           config.str_param.c_str(), 
           config.bool_param?"True":"False", 
           config.size);
   robot_speed = config.double_param;
}

int main(int argc, char **argv) 
{
   ros::init(argc, argv, "dynamic_tutorials");
   //创建了一个参数动态配置的服务端实例
   //参数配置的类型与配置文件中描述的类型相同

   ros::NodeHandle(nh);
   dynamic_reconfigure::Server<upros_message::TutorialsConfig> server;
   dynamic_reconfigure::Server<upros_message::TutorialsConfig>::CallbackType f;

   ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
   
   //绑定回调
   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);

   ROS_INFO("Spinning node");
   ros::Rate rate(10);
   while(ros::ok())
   {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = robot_speed;
      cmd_pub_.publish(cmd_vel);
      ros::spinOnce();
      rate.sleep();

   }

   return 0;
}
