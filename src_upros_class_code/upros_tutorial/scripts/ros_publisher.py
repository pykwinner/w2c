#!/usr/bin/env python3

import rospy
from upros_message.msg import MyMessage

rospy.init_node('my_publisher')

# 定义一个发布者对象
pub = rospy.Publisher('my_topic', MyMessage, queue_size=10)

while not rospy.is_shutdown():
    # 创建一个消息对象并填充数据
    msg = MyMessage()
    msg.key = 1
    msg.value = 'Hello, ROS!'
    # 发布消息
    pub.publish(msg)
    #沉睡1秒
    rospy.sleep(1.0)
