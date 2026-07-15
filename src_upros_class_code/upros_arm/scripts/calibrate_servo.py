#!/usr/bin/env python3
import rospy
from zoo_bringup.msg import SingleServo,MultipleServo 
from std_msgs.msg import Int16



class ServoCalibrate:
    def __init__(self):
        rospy.init_node('servo_calibrate_node',anonymous=True)

        param_bias_1 = rospy.get_param('/calibrate_servo/servo_bias_1', 0)
        param_bias_2 = rospy.get_param('/calibrate_servo/servo_bias_2', 0)
        param_bias_3 = rospy.get_param('/calibrate_servo/servo_bias_3', 0)
        param_bias_4 = rospy.get_param('/calibrate_servo/servo_bias_4', 0)
        param_bias_5 = rospy.get_param('/calibrate_servo/servo_bias_5', 0)
        param_bias_6 = rospy.get_param('/calibrate_servo/servo_bias_6', 0)

        self.param_bias_list = [param_bias_1, param_bias_2, param_bias_3, param_bias_4, param_bias_5, param_bias_6]

        print (self.param_bias_list)

        rospy.Subscriber('/command', Int16, self.command_callback)

        self.multiple_servo_pub = rospy.Publisher('/multiple_servo_topic', MultipleServo, queue_size = 5)

        rospy.spin()
             
        
    def servo_mid(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1,6):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 50
            single_servo.Target_position_Angle = 0 + self.param_bias_list[i-1]
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)
    

    def command_callback(self,msg):
        cmd = msg.data
        if cmd == 1:
            self.servo_mid()           

 
if __name__ == "__main__":
    servo = ServoCalibrate()
