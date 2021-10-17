#!/usr/bin/env python3

import rospy

from RoboticArmClass import RoboticArm
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
from inverse_problem_srv.srv import point_cmd

class ToPointController():
    def __init__(self) -> None:
        rospy.Service('/target_point', point_cmd, self.targetPointCb)
        rospy.Subscriber('/joint_state_goal/state', Int16, self.armStateCb)
        self.joint_state_pub = rospy.Publisher('/joint_state_goal', JointState, queue_size=10)
        self.robotic_arm = RoboticArm()
        self.arm_state = 0

    def targetPointCb(self, msg):
        x,y,z,pith,roll = self.parseMsg(msg.point)

        result, joint_state = self.robotic_arm.InversProblem(x,y,z,pith,roll)

        rospy.loginfo("{0} {1}".format(result, joint_state))
        
        if result:
            new_joint_msg = JointState()
            new_joint_msg.position = joint_state
            new_joint_msg.header.stamp = rospy.Time.now()
            
            rospy.loginfo('Publishing cmd')
            while(self.arm_state == 0):
                self.joint_state_pub.publish(new_joint_msg)
                rospy.sleep(0.1)

            rospy.loginfo("Wait...")
            while(self.arm_state == 1):
                rospy.sleep(0.5)

            return True
        else:
            return False
    
    def armStateCb(self, msg):
        self.arm_state = msg.data

    def parseMsg(self, point):
        try:
            coord_list = point.split()
            x = float(coord_list[0])
            y = float(coord_list[1])
            z = float(coord_list[2])
            pith = float(coord_list[3])
            roll = -float(coord_list[4])
            return x,y,z,pith,roll
        except ValueError:
            rospy.logerr('Input Error')

if __name__=="__main__":
    rospy.init_node('to_point_node')

    to_point_controller = ToPointController()

    rospy.spin()