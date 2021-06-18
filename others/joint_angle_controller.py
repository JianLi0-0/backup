import rospy
from std_msgs.msg import Float64MultiArray
from rospy.client import spin
from sensor_msgs.msg import JointState
import time
import math
import numpy as np
joint_angle = Float64MultiArray()

# target_joint_angle = np.array([-0.008477036152974904, -1.1687305730632325, -2.3112632274627686, -1.4607807409814377, 0, 1.893333077430725])
target_joint_angle = np.array([-0.008477036152974904, -1.1687305730632325, -1.9112632274627686, -1.4607807409814377, 1.5298510789871216, 1.7201569080352783])

def joint_states_cb(msg):
    temp = list(msg.position)
    temp[0] = msg.position[2]
    temp[2] = msg.position[0]
    joint_angle.data = tuple(temp)

pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
sub = rospy.Subscriber('/joint_states', JointState, callback=joint_states_cb, queue_size=1)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1000) # 10hz
time.sleep(1)
while not rospy.is_shutdown():
    joint_vel = Float64MultiArray()
    temp = target_joint_angle - np.array(list(joint_angle.data))
    joint_vel.data = tuple(temp*10)
    pub.publish(joint_vel)
    rate.sleep()
    # print(np.linalg.norm(temp))
    if np.linalg.norm(temp) < 1e-7:
        exit()
rospy.spin()