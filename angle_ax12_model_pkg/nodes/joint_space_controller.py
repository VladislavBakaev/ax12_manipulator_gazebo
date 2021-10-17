#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from std_msgs.msg import Int16

class JointSpaceController():
    def __init__(self) -> None:
        rospy.Subscriber('/joint_states', JointState, self.currentJointStateCb)
        rospy.Subscriber('/joint_state_goal', JointState, self.goalJointStateCb)

        self.cmd_joint_publisher = rospy.Publisher('/angle_ax12_controller/command', JointTrajectory, queue_size=10)
        self.feedback_publisher = rospy.Publisher('/joint_state_goal/state', Int16, queue_size=10)

        self.joint_count = 5
        self.error_tolerancy = 0.05
        self.max_joint_velocity = 1.0
        self.current_joint_state = JointState()
        self.goal_joint_state = JointState()

        rospy.Timer(rospy.Duration(0.05), self.feedbackUpdate)

    def feedbackUpdate(self, event):
        error = self.getJointError()
        
        if len(error)<self.joint_count:
            return
    
        if (max(error)>self.error_tolerancy):
            data = 1
        else:
            data = 0
        
        new_msg = Int16()
        new_msg.data = data
        self.feedback_publisher.publish(new_msg)

    def currentJointStateCb(self, msg) -> None:
        if len(msg.effort) > 0:
            self.current_joint_state.name = msg.name[2:]
            self.current_joint_state.position = msg.position[2:]
            self.current_joint_state.velocity = msg.velocity[2:]
        
        else:
            self.current_joint_state.name = msg.name
            self.current_joint_state.position = msg.position
            self.current_joint_state.velocity = msg.velocity
        
        if len(self.goal_joint_state.position)==0:
            self.goal_joint_state = self.current_joint_state

    def goalJointStateCb(self, msg) -> None:
        if len(msg.position) < self.joint_count:
            rospy.logerr("Invalide count of cmd joint")
            return
        else:
            self.goal_joint_state = msg

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()

        point.positions = self.goal_joint_state.position
        point.time_from_start = rospy.Duration(self.getExecutionTime())

        trajectory.joint_names = self.current_joint_state.name
        trajectory.header.stamp = rospy.Time.now()
        trajectory.points = [point]

        self.cmd_joint_publisher.publish(trajectory)
    
    def getExecutionTime(self) -> float:
        err = np.array(self.getJointError())
        time_list = err/self.max_joint_velocity
        max_time = max(time_list)
        return max_time
        
    def getJointError(self) -> list:
        current_js_np = np.array(self.current_joint_state.position)
        goal_js_np = np.array(self.goal_joint_state.position)
        error = goal_js_np - current_js_np
        return error.tolist()

if __name__=="__main__":
    rospy.init_node('joint_space_controller_node')

    joint_state_controller = JointSpaceController()

    rospy.spin()