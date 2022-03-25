#! /usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint


if __name__=='__main__':

 roscpp_initialize(sys.argv)
 rospy.init_node('moveit_py_demo', anonymous=True)

 scene = PlanningSceneInterface()
 robot = RobotCommander()
 arm_torso = MoveGroupCommander("arm_torso")
 end_effector_link = arm_torso.get_end_effector_link()  

 print "======= PICK"
 arm_torso.pick("part")
 rospy.sleep(2)  

 p = PoseStamped()
 p.header.frame_id = "base_footprint"
 p.pose.position.x = 0.8
 p.pose.position.y = -0.3
 p.pose.position.z = 0.7
 p.pose.orientation.w = 1.0

 print "======= PLACE"
 arm_torso.place("part",p)
