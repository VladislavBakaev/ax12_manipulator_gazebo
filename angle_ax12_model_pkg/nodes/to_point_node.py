#!/usr/bin/env python3

import rospy

from RoboticArmClass import RoboticArm
from inverse_problem_srv.srv import point_cmd

class ToPointController():
    def __init__(self) -> None:
        rospy.Service('/target_point', point_cmd, self.targetPointCb)
        self.robotic_arm = RoboticArm()

    def targetPointCb(self, msg):
        x,y,z,pith,roll = self.parseMsg(msg.point)

        result, joint_state = self.robotic_arm.InversProblem(x,y,z,pith,roll)

        rospy.loginfo("{0} {1}".format(result, joint_state))

        return True

    def parseMsg(self, point):
        try:
            coord_list = point.split()
            x = float(coord_list[0])
            y = float(coord_list[1])
            z = float(coord_list[2])
            pith = -1.57
            roll = -float(coord_list[3])
            return x,y,z,pith,roll
        except ValueError:
            rospy.logerr('Input Error')

if __name__=="__main__":
    rospy.init_node('to_point_node')

    to_point_controller = ToPointController()

    rospy.spin()