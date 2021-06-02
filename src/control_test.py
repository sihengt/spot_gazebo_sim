#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry


def timer_callback(msg):
    global odom
    odom.header.stamp = rospy.Time.now()

    odom_pub.publish(odom)
    

if __name__ == '__main__':
    rospy.init_node('control_test')

    odom = Odometry()
    odom.header.frame_id = 'world'
    odom.child_frame_id = 'world'
    odom.pose.pose.position.z = 1
    odom.pose.pose.orientation.w = 1

    timer = rospy.Timer(rospy.Duration(1./50.), timer_callback)
    
    odom_pub = rospy.Publisher('/uav3/tracking_point', Odometry, queue_size=10)


    rospy.spin()
