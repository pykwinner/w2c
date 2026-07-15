#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LaserFilterNode:
    def __init__(self):
        rospy.init_node('laser_filter', anonymous=True)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laserScanCallback)
        self.pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=1)

    def laserScanCallback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = -90 * (3.141592653589793 / 180)  # -90 degrees in radians
        filtered_scan.angle_max = 90 * (3.141592653589793 / 180)   # 90 degrees in radians
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        start_index = int((filtered_scan.angle_min - msg.angle_min) / msg.angle_increment)
        end_index = int((filtered_scan.angle_max - msg.angle_min) / msg.angle_increment)

        for i in range(start_index, end_index):
            filtered_scan.ranges.append(msg.ranges[i])

        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    node = LaserFilterNode()
    rospy.spin()