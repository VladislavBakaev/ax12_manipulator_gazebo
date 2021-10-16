#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState

class GripperPlugin():
    def __init__(self) -> None:
        rospy.Service('/gripper_cmd', SetBool, self.gripperCmdCb)
        rospy.Subscriber('/joint_states', JointState, self.gripperStateCb)

        self.left_gripper_pub = rospy.Publisher('/gripper_position/command', Float64, queue_size=10)
        self.right_gripper_pub = rospy.Publisher('/gripper_sub_position/command', Float64, queue_size=10) 
        self.state_publisher = rospy.Publisher('/gripper_state', JointState, queue_size=10)

        rospy.Timer(rospy.Duration(0.5), self.publishGripperCmd)

        self.gripper_state = JointState()
        self.gripper_state.position = [0]
        self.gripper_state.velocity = [0]
        self.gripper_state.effort = [0]

        self.left_gripper_cmd = Float64()
        self.right_gripper_cmd = Float64()

        self.open_pose_gripper = 0.0
        self.close_pose_gripper = 0.026

    def gripperCmdCb(self, msg) -> None:
        if msg.data:
            self.left_gripper_cmd.data = self.close_pose_gripper
            self.right_gripper_cmd.data = self.close_pose_gripper
        else:
            self.left_gripper_cmd.data = self.open_pose_gripper
            self.right_gripper_cmd.data = self.open_pose_gripper
        
        return True, ''

    def gripperStateCb(self, msg) -> None:
        if len(msg.effort) > 0:
            self.gripper_state.position[0] = msg.position[0]
            self.gripper_state.velocity[0] = msg.velocity[0]
            self.gripper_state.effort[0] = msg.effort[0]

            self.gripper_state.header.stamp = rospy.Time.now()
            self.state_publisher.publish(self.gripper_state)
    
    def publishGripperCmd(self, event):
        self.left_gripper_pub.publish(self.left_gripper_cmd)
        self.right_gripper_pub.publish(self.right_gripper_cmd)

if __name__=="__main__":
    rospy.init_node('gripper_plugin')

    gripper = GripperPlugin()

    rospy.spin()