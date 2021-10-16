#!/usr/bin/env python3
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('test')

msg = JointTrajectory()
point1 = JointTrajectoryPoint()
point2 = JointTrajectoryPoint()

msg.header.stamp = rospy.Time.now()

msg.joint_names = ['ang12ax_joint1', 'ang12ax_joint2', 'ang12ax_joint3', 'ang12ax_joint4', 'ang12ax_joint5']

point1.positions = [0,0,0,0,0]
point1.time_from_start = rospy.Duration(2.0)

point2.positions = [1.57,0,1.57,0]
point2.time_from_start = rospy.Duration(4.0)
msg.points = [point1, point2]

pub = rospy.Publisher('/angle_ax12_controller/command', JointTrajectory, queue_size=10)

for _ in range(50):
    pub.publish(msg)
    rospy.sleep(0.01)