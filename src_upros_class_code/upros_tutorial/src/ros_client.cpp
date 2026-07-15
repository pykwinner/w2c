#include "ros/ros.h"
#include "upros_message/MyServiceMsgRequest.h"
#include "upros_message/MyServiceMsgResponse.h"
#include "upros_message/MyServiceMsg.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_service_client");
    ros::NodeHandle nh;

    // 创建一个 ROS 服务客户端，请求 my_service 服务
    ros::ServiceClient client = nh.serviceClient<upros_message::MyServiceMsg>("my_service");

    // 创建一个请求消息
    upros_message::MyServiceMsg srv;
    srv.request.input = 42;
    if (client.call(srv)) {
        ROS_INFO("Service response: %ld", (long int) srv.response.output);  // 输出响应结果
    } else {
        ROS_ERROR("Failed to call service my_service");
        return 1;
    }

    return 0;
}
