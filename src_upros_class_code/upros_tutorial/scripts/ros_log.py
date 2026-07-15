#!/usr/bin/env python3

import rospy

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('ros_logging_example')

    # 打印不同级别的日志消息
    rospy.logdebug("This is a DEBUG message.")
    rospy.loginfo("This is an INFO message.")
    rospy.logwarn("This is a WARNING message.")
    rospy.logerr("This is an ERROR message.")
    rospy.logfatal("This is a FATAL message.")

    # 运行ROS事件循环
    rospy.spin()
