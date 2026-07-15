#!/usr/bin/env python3

import rospy
import actionlib

from upros_message.msg import MyActionAction,MyActionGoal,MyActionResult,MyActionFeedback

class MyActionServer(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MyActionAction, execute_cb=self.executeCB, auto_start = False)
        self._as.start()
        self._feedback = MyActionFeedback()
        self._result = MyActionResult()

    def executeCB(self, goal):
        r = rospy.Rate(1)
        success = True
        # 执行动作
        for i in range(1, 11):
            if self._as.is_preempt_requested():
                rospy.loginfo('{}: Preempted'.format(self._action_name))
                self._as.set_preempted()
                success = False
                break

            self._feedback.progress = i * 10
            rospy.loginfo('{}: Executing, progress = {}'.format(self._action_name, self._feedback.progress))
            self._as.publish_feedback(self._feedback)

            r.sleep()

        # 发送结果
        if success:
            self._result.success = True
            rospy.loginfo('{}: Succeeded'.format(self._action_name))
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('my_server')
    server = MyActionServer('my_action')
    rospy.spin()
