#! /usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from my_grasping.srv import *
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose




import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

#import moveit_msgs.msg
import moveit_commander
import tf  # just for transformations, if you're doing stuff like broadcasting then use tf2_ros

import math

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Point
import copy

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    client = SimpleActionClient("play_motion", PlayMotionAction)
    client.wait_for_server()
    rospy.loginfo("...connected.")

    # INSPECT SURROUNDINGS TO BUILD OCTOMAP
    rospy.loginfo("Grasping demo...")
    goal = PlayMotionGoal()
    goal.motion_name = 'home'#'inspect_surroundings'
    goal.skip_planning = True
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(10.0))

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("arm_torso")
    print "============ Robot Groups:"
    print robot.get_group_names()
    print right_arm.get_named_targets()
    right_gripper = MoveGroupCommander("gripper")
    print right_gripper.get_named_targets()
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("MApart")
    scene.remove_attached_object()  # removes all previously attached objects


    # FILTER, SEGMENT AND CLUSTER TO FIND 3D OBJECT BOUNDING BOX
    #rospy.wait_for_service('find_pick')
    #try:
#        pick_as = rospy.ServiceProxy('find_pick', GetTargetPose)
#        resp1 = pick_as()#
#        rospy.loginfo("Got target pose: " + str(resp1.target_pose))
#    except rospy.ServiceException as e:
#        print("Service call failed: %s"%e)

    #right_arm.set_named_target("home")
    #right_arm.go()

    #right_gripper.set_named_target("open_gripper")
    #right_gripper.go()

    rospy.sleep(1)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # add a table
    p.pose.position.x = 0.6
    p.pose.position.y = -0.2
    p.pose.position.z = 0.24
    #scene.add_box("table", p, (0.5, 1.5, 0.48))

    # add an object to be grasped
    p.pose.position.x = 0.5# 0.605
    p.pose.position.y = 0.5# -0.12
    p.pose.position.z = 0.5# 0.35
    scene.add_box("MApart", p, (0.05, 0.05, 0.05))

    rospy.sleep(1)
    rospy.loginfo("added objects")

    # define a list of grasps, for now a single grasp
    grasps = [Grasp()]

    grasps[0].id = "test"

    # set the pose of the Pose of the end effector in which it should attempt grasping
    grasps[0].grasp_pose = PoseStamped()
    grasps[0].grasp_pose.header.frame_id = "base_footprint"
    #grasps[0].grasp_pose.pose.position.x = 0.5
    #grasps[0].grasp_pose.pose.position.y = 0.5
    #grasps[0].grasp_pose.pose.position.z = 0.5
    #grasps[0].grasp_pose.pose.orientation.x = 0
    #grasps[0].grasp_pose.pose.orientation.y = 0
    #grasps[0].grasp_pose.pose.orientation.z = 0
    #grasps[0].grasp_pose.pose.orientation.w = 1

    #q = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))
    q = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
    grasps[0].grasp_pose.pose = Pose(Point(0.5, 0.5, 0.73), Quaternion(*q))
    rospy.sleep(2)

#      THIS IS SOMETHING THE TIAGO TUTORIALS DID, MAYBE YOU GOTTA DO IT TOO
#            header = Header()
#        header.frame_id = self._grasp_pose_frame_id  # base_footprint
#        q = [pose.orientation.x, pose.orientation.y,
#             pose.orientation.z, pose.orientation.w]
#        # Fix orientation from gripper_link to parent_link (tool_link)
#        fix_tool_to_gripper_rotation_q = quaternion_from_euler(
#            math.radians(self._fix_tool_frame_to_grasping_frame_roll),
#            math.radians(self._fix_tool_frame_to_grasping_frame_pitch),
#            math.radians(self._fix_tool_frame_to_grasping_frame_yaw)
#        )
#        q = quaternion_multiply(q, fix_tool_to_gripper_rotation_q)
#        fixed_pose = copy.deepcopy(pose)
#        fixed_pose.orientation = Quaternion(*q)
#
#        g.grasp_pose = PoseStamped(header, fixed_pose)
## THIS IS ANOTHER POSSIBLE FIX FROM SOME GUY ON GITHUB
# org_quat = # get your grasp orientation
# rot_quat = Quaternion(0.7071, 0.7071, 0, 0)  # 90* around X axis (W, X, Y, Z)
# grasp_quat = rot_quat * org_quat

    # define the pre-grasp approach, this is used to define the direction from which to approach the object and the distance to travel
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint"
    grasps[0].pre_grasp_approach.direction.vector.x = 0.0
    grasps[0].pre_grasp_approach.direction.vector.y = 0.0
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0
    grasps[0].pre_grasp_approach.min_distance = 0.05
    grasps[0].pre_grasp_approach.desired_distance = 0.1

    # define the pre-grasp posture, this defines the trajectory position of the joints in the end effector group before we go in for the grasp
    # WE WANT TO OPEN THE GRIPPER, YOU CAN TAKE SOME OF THIS AND MAKE INTO CLOSE GRIPPER FUNCTION
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "arm_tool_link"
    pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = [0.038, 0.038]
    jtpoint.time_from_start = rospy.Duration(2.0)
    pre_grasp_posture.points.append(jtpoint)
    grasps[0].pre_grasp_posture = pre_grasp_posture

    # set the grasp posture, this defines the trajectory position of the joints in the end effector group for grasping the object
    # WE WANT TO CLOSE THE GRIPPER
    grasp_posture = copy.deepcopy(pre_grasp_posture)
    grasp_posture.points[0].time_from_start = rospy.Duration(2.0 + 1.0)
    jtpoint2 = JointTrajectoryPoint()
    jtpoint2.positions = [0.015, 0.015]
    jtpoint2.time_from_start = rospy.Duration(2.0 + 1.0 + 3.0)
    grasp_posture.points.append(jtpoint2)
    grasps[0].grasp_posture = grasp_posture



    # set the post-grasp retreat, this is used to define the direction in which to move once the object is grasped and the distance to travel.
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint"
    grasps[0].post_grasp_retreat.direction.vector.x = 0.0
    grasps[0].post_grasp_retreat.direction.vector.y = 0.0
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0
    grasps[0].post_grasp_retreat.desired_distance = 0.25
    grasps[0].post_grasp_retreat.min_distance = 0.01

    # links_to_allow_contact: ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]
    #grasps[0].allowed_touch_objects = ["table", "MApart"]  # THIS LEFT EMPTY ON THINGY
    #grasps[0].links_to_allow_contact = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]
    #Don't restrict contact force
    #grasps[0].max_contact_force = 0.0
    print "max contact force", grasps[0].max_contact_force

    # append the grasp to the list of grasps
    ##grasps.append(g)

    rospy.sleep(2)

    # pick the object
    right_arm.pick("MApart", grasps)

    rospy.spin()
    roscpp_shutdown()








#import rospy
#import actionlib
#from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
#from sensor_msgs.msg import JointState

if __name__ == "__main__" and False:
    rospy.init_node("grasp_demo")
    rospy.loginfo("Waiting for play_motion...")
    client = SimpleActionClient("play_motion", PlayMotionAction)
    client.wait_for_server()
    rospy.loginfo("...connected.")

    # INSPECT SURROUNDINGS TO BUILD OCTOMAP
    rospy.loginfo("Grasping demo...")
    goal = PlayMotionGoal()
    goal.motion_name = 'inspect_surroundings'
    goal.skip_planning = True
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(10.0))

    rospy.loginfo("Setting publishers to torso and head controller...")
    torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
    head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
    rospy.loginfo("movoing to next stage...")

    # FILTER, SEGMENT AND CLUSTER TO FIND OBJECT CENTROID
    rospy.wait_for_service('find_pick')
    try:
        pick_as = rospy.ServiceProxy('find_pick', GetTargetPose)
        resp1 = pick_as()
        rospy.loginfo("Got target pose: " + str(resp1.target_pose))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # ADD OBJECT TO PLANNING SCENE
    scene = PlanningSceneInterface()
    # create PoseStamped
    pose = PoseStamped()
    pose.header = std_msgs.msg.Header()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base_footprint"
    pose.pose = Pose()
    pose.pose = resp1.target_pose  # MAYBE SHOULD DO A DEEP COPY HERE



    scene.add_box("MY_part", pose, (1,1,1))
    # moveit_msgs::CollisionObject collision_object;
    # collision_object.header.frame_id = "base_footprint";
    # collision_object.id = "cylinder";
    #
    # primitive = shape_msgs.SolidPrimitive()
    # primitive.type = primitive.CYLINDER
    # primitive.dimensions.resize(2)
    # # Setting height of cylinder
    # primitive.dimensions[0] = resp1.target_pose.position.z
    # # Setting radius of cylinder
    # primitive.dimensions[1] = 0.2
    #
    # collision_object.primitives.push_back(primitive);
    # collision_object.primitive_poses.push_back(cylinder_pose);
    # collision_object.operation = collision_object.ADD;
    # scene.applyCollisionObject(collision_object);


    # ADJUST POSE TO MAKE TOP DOWN AND TAKE INTO ACCOUNT LINK LENGTH
    resp1.target_pose.position.z += 0.5
    resp1.target_pose.orientation.y = 0.70682518
    resp1.target_pose.orientation.w = 0.70738827
    rospy.loginfo("WOOP Got target pose: " + str(resp1.target_pose))
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("arm_torso")
    move_group.set_pose_target(resp1.target_pose)
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    rospy.spin()

if __name__=='__main__' and False:

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    # Look around to build octomap
    play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Created /play_motion client waiting for server")
    if not play_m_as.wait_for_server(rospy.Duration(20)):
        rospy.logerr("Could not connect to /play_motion AS")
        exit()
        rospy.loginfo("Connected!")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'inspect_surroundings'
        pmg.skip_planning = False
        play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")


        rospy.loginfo("Setting publishers to torso and head controller...")
        torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)



        #rospy.wait_for_service('find_pick')
        #pick_as = rospy.ServicePsroxy('find_pick', GetTargetPose)
        #pa = pick_as()
        #part_pose = rospy.wait_for_message('part', Pose)
        #rospy.loginfo("Got: " + str(part_pose))


        rospy.wait_for_service('find_pick')
        try:
            pick_as = rospy.ServiceProxy('find_pick', GetTargetPose)
            resp1 = pick_as()
            rospy.loginfo("Got target pose: " + str(resp1.target_pose))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



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
