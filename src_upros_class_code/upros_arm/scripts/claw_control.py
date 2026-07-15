#!/usr/bin/env python3
import rospy
from zoo_bringup.msg import SingleServo,MultipleServo 
from std_msgs.msg import Int16



class ClawController:
    def __init__(self):
        rospy.init_node('claw_control_node',anonymous=True)

        rospy.Subscriber('/command', Int16, self.command_callback)

        self.single_servo_pub = rospy.Publisher('/single_servo_topic', SingleServo, queue_size = 5)

        rospy.spin()
             
        
    def close_claw(self):
        single_servo = SingleServo()
        single_servo.ID = 6
        single_servo.Rotation_Speed = 50
        single_servo.Target_position_Angle = -580
        self.single_servo_pub.publish(single_servo)
    
    def open_claw(self):
        single_servo = SingleServo()
        single_servo.ID = 6
        single_servo.Rotation_Speed = 50
        single_servo.Target_position_Angle = 580
        self.single_servo_pub.publish(single_servo)

    def command_callback(self,msg):
        cmd = msg.data
        if cmd == 1:
            self.close_claw()    
        elif cmd == 2:
            self.open_claw()        

 
if __name__ == "__main__":
    servo = ClawController()
