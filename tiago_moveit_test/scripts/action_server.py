#! /usr/bin/env python

import rospy
import actionlib
import tiago_moveit_test.msg


import sys
#import copy
import moveit_commander
import moveit_msgs.msg  # for display_trajectory_publisher

# because of transformations
import tf


class MoveTorsoArmActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = tiago_moveit_test.msg.MoveTorsoArmFeedback()
    _result = tiago_moveit_test.msg.MoveTorsoArmResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, tiago_moveit_test.msg.MoveTorsoArmAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm_torso"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", group_names)

        move_group.set_pose_target(goal.goal_pose)

        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # publish info to the console for the user
        #rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        rospy.loginfo('HI')

        # MAYBE CHANGE TO SOMETHING THAT CHECKS IF SUCCESSFUL OR JUST MAKE SERVICE INSTEAD OF ACTION
        self._result.success = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('move_torso_arm_as')
    server = MoveTorsoArmActionServer(rospy.get_name())
    rospy.spin()
