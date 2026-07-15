#!/usr/bin/env python3

import rospy
from upros_message.srv import MyServiceMsg

def handle_my_service(req):
    rospy.loginfo("Service request: %d", req.input)
    return req.input * 2  # 返回输出结果

rospy.init_node("my_service_server")
service = rospy.Service("my_service", MyServiceMsg, handle_my_service)
rospy.loginfo("Ready to handle requests")
rospy.spin()
