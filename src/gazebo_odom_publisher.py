#!/usr/bin/python                                                                                                                                                                        
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


def states_callback(msg):
    odom = Odometry()

    for i in range(len(msg.name)):
        name = msg.name[i]
        if name == model_name:
            pose = msg.pose[i]
            twist = msg.twist[i]

            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'world'
            odom.pose.pose = pose
            odom.twist.twist = twist

            odom_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('state_publisher')
    
    model_name = rospy.get_param('~model_name', 'uav')
    
    gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, states_callback)

    odom_pub = rospy.Publisher('gazebo_odometry', Odometry, queue_size=10)

    rospy.spin()
