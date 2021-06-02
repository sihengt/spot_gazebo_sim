#!/usr/bin/python                                                                                                                                                                        
import rospy
from sensor_msgs.msg import LaserScan, Range
import math
import numpy as np

range_pub = None

def laser_scan_callback(msg):
    r = Range()
    r.header = msg.header
    r.radiation_type = Range.INFRARED
    r.min_range = msg.range_min
    r.max_range = msg.range_max
    r.field_of_view = math.atan2(math.sin(msg.angle_max - msg.angle_min), math.cos(msg.angle_max - msg.angle_min))
    r.range = np.mean(msg.ranges)
    range_pub.publish(r)

if __name__ == '__main__':
    rospy.init_node('laser_scan_to_range')

    rospy.Subscriber('laser', LaserScan, laser_scan_callback)

    range_pub = rospy.Publisher('range', Range, queue_size=10)

    rospy.spin()
