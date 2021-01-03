import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=1)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1) # 10hz
# while not rospy.is_shutdown():
p1 = JointTrajectoryPoint()
p2 = JointTrajectoryPoint()
p1.positions=[0,0,0,0,0,0]
p1.time_from_start.secs = 2
p1.velocities=[1.0,-1.0,1.0,1.0,1.0,1.0]
p2.positions=[-0.268793, -1.82578, 1.84404, -2.05047, 0.0987335, -0.408469]
p2.time_from_start.secs = 5
p2.velocities=[1.0,-1.0,1.0,1.0,1.0,1.0]
t = JointTrajectory()
t.joint_names = ["shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
t.points.append(p1)
t.points.append(p2)
rate.sleep()
pub.publish(t)
rate.sleep()