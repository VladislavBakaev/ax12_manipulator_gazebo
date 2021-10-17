#!/usr/bin/env python3
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

rospy.init_node('test')

msg = JointState()
# msg = JointTrajectory()
# point1 = JointTrajectoryPoint()
# point2 = JointTrajectoryPoint()

msg.header.stamp = rospy.Time.now()
msg.position = [1.57,0,1.57,0,0]

# msg.joint_names = ['ang12ax_joint1', 'ang12ax_joint2', 'ang12ax_joint3', 'ang12ax_joint4', 'ang12ax_joint5']

# point1.positions = [0,0,0,0,0]
# point1.time_from_start = rospy.Duration(0.0)

# point2.positions = [1.57,0,1.57,0]
# point2.time_from_start = rospy.Duration(4.0)
# msg.points = [point1, point2]

pub = rospy.Publisher('/joint_state_goal', JointState, queue_size=10)

for _ in range(1):
    pub.publish(msg)
    rospy.sleep(0.01)