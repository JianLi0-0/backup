#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(100) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['link1', 'link11', 'link2', 'link22','link3,','link33', 'link4', 'link44', 'link5', 'link51', 'link52',
'link53']
    hello_str.position = [0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    rate.sleep()

if __name__ == '__main__':
    try:
        while True:
            talker()
    except rospy.ROSInterruptException:
        pass
