#!/usr/bin/env python3

import rospy
import actionlib

from upros_message.msg import MyActionAction,MyActionGoal,MyActionResult,MyActionFeedback

def feedback_cb(feedback):
    rospy.loginfo('Progress: {}'.format(feedback.progress))

if __name__ == '__main__':
    rospy.init_node('my_client')
    client = actionlib.SimpleActionClient('my_action', MyActionAction)
    client.wait_for_server()

    # 创建动作目标
    goal = MyActionGoal()
    
    goal.object_name = 'world'

    rospy.loginfo('Sending goal...')

    # 发送目标并等待结果
    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    # 输出结果
    result = client.get_result()
    if result.success:
        rospy.loginfo('Action succeeded')
    else:
        rospy.loginfo('Action failed')
