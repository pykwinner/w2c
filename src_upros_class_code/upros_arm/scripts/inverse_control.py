#!/usr/bin/env python3
import rospy
from zoo_bringup.msg import SingleServo,MultipleServo 
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from math import *

l0 = 110
l1 = 61
l2 = 81
l3 = 200

servo_angle = [0,0,0,0,0,0]#记录1,2,3,4,5,6角度

class Inverse_Control:
    def __init__(self):
        rospy.init_node('servo_calibrate_node',anonymous=True)

        param_bias_1 = rospy.get_param('/inverse_control/servo_bias_1', 0)
        param_bias_2 = rospy.get_param('/inverse_control/servo_bias_2', 0)
        param_bias_3 = rospy.get_param('/inverse_control/servo_bias_3', 0)
        param_bias_4 = rospy.get_param('/inverse_control/servo_bias_4', 0)
        param_bias_5 = rospy.get_param('/inverse_control/servo_bias_5', 0)
        param_bias_6 = rospy.get_param('/inverse_control/servo_bias_6', 0)

        self.param_bias_list = [param_bias_1, param_bias_2, param_bias_3, param_bias_4, param_bias_5, param_bias_6]

        print (self.param_bias_list)

        rospy.Subscriber('/goal', Point, self.goal_callback)

        rospy.Subscriber('/command', Int16, self.command_callback)

        self.multiple_servo_pub = rospy.Publisher('/multiple_servo_topic', MultipleServo, queue_size = 5)

        self.inverse_calcate(-150, 205, 72)

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
    
    def inverse_move(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1,6):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 50
            single_servo.Target_position_Angle = servo_angle[i-1] + self.param_bias_list[i-1]
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)
         
    def command_callback(self,msg):
        cmd = msg.data
        if (cmd == 5):
             self.inverse_move()
     
    def goal_callback(self,msg):
         goal_x = int(msg.x)
         goal_y = int(msg.y)
         goal_z = int(msg.z)
         self.inverse_calcate(goal_x, goal_y, goal_z)

    def inverse_calcate(self, x, y, z):
         best_alpha = 0
         alpha_list = []
         for i in range(-60, 0):
              if(self.inverse_analysis(x, y, z, i) == 0):
                   alpha_list.append(i)
         if(len(alpha_list)>0):
             best_alpha = min(alpha_list)
             flag = 1
         if(flag):
              self.inverse_analysis(x,y,z,best_alpha)
              print(str(best_alpha) + ' Final Result:')
              print(servo_angle)
              

    def inverse_analysis(self, dis_x, dis_y, dis_z, alpha):
         #calcate theta1
         if(dis_x==0 and dis_y!=0):
              theta1 = 0
         elif(dis_x>0 and dis_y == 0):
              theta1 = 90
         elif(dis_x<0 and dis_y==0):
              theta1 = -90
         else:
              theta1 = atan(dis_x/dis_y)*180.0/pi
         
         #calcate total_length in y-z axis
         y123 = sqrt(dis_y*dis_y + dis_x*dis_x)   

         y12 = y123 - l3*cos(alpha*pi/180.0)
         print('y12: ' + str(y12))
         z12 = dis_z - l0 -l3*sin(alpha*pi/180)
         print('z12: ' + str(z12))

         if(z12 < - l0):
              return 1
         if(sqrt(y12*y12 + z12*z12) > (l1+l2)):
              return 2

         ccc = acos(y12 / sqrt(y12*y12 + z12*z12))
         print('ccc: ' + str(ccc*180.0/pi))
         bbb = acos((y12*y12+z12*z12+l1*l1-l2*l2)/(2*l1*sqrt(y12*y12 + z12*z12)))
         if(bbb > 1 or bbb < -1) :
             return 3
         print('bbb: ' + str(bbb*180.0/pi))

         aaa = acos((y12*y12+z12*z12+l2*l2-l1*l1)/(2*l2*sqrt(y12*y12 + z12*z12)))
         print('aaa: ' + str(aaa*180.0/pi))
         if(aaa > 1 or aaa < -1) :
             return 4

         if(z12<0):
              zf_flag = -1
         else:
              zf_flag = 1

         theta2 = pi/2 - (ccc*zf_flag + bbb)
         theta2 = theta2 * 180.0 /pi
         if (theta2 > 135.0 or theta2 < -135.0) :
             return 5

         theta3 = aaa + bbb
         theta3 = theta3 * 180.0/pi
         if (theta3 > 135.0 or theta3 < -135.0) :
             return 6

         theta4 = pi/2 - alpha*pi/180.0 - theta2*pi/180.0 - theta3*pi/180.0
         theta4 = theta4 * 180.0/pi
         if (theta4 > 135.0 or theta4 < -135.0) :
             return 7

         servo_angle[0] = int(theta1 * 10)
         servo_angle[1] = int(-theta2 * 10)
         servo_angle[2] = int(-theta3 * 10)
         servo_angle[3] = int(-theta4 * 10)

         return 0

 
if __name__ == "__main__":
    inverse = Inverse_Control()
