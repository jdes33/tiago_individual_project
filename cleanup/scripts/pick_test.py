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


from haf_grasping.srv import *
from std_msgs.msg import String
import rospkg

haf_ready = False
current_haf_grasp = None

def process_haf_grasp(data):
    grasp_hypothesis = data.data.split(" ")
    print grasp_hypothesis
    grasp = {"val":0, "gp1":[1,2,3], "gp2":[2,3,4], "av":[1,1,1], "gcp":[5,5,5], "roll":[30]}

    grasp["val"] = int(grasp_hypothesis[0])
    grasp["gp1"] = [float(e) for e in grasp_hypothesis[1:4]]
    grasp["gp2"] = [float(e) for e in grasp_hypothesis[4:7]]
    grasp["av"] = [float(e) for e in grasp_hypothesis[7:10]]
    grasp["gcp"] = [float(e) for e in grasp_hypothesis[10:13]]
    grasp["roll"] = float(grasp_hypothesis[13])

    global haf_ready, current_haf_grasp
    haf_ready = True
    current_haf_grasp = grasp

    print "sup"
    print grasp
    print "hup"



if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    rospack = rospkg.RosPack()  # will be used to get package paths

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

    rospy.loginfo("Creating publishers to torso and head controller...")
    torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1, latch=True)
    head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1, latch=True)
    rospy.loginfo("Creating publisher for haf_grasping")
    haf_pub = rospy.Publisher('/haf_grasping/input_pcd_rcs_path', String, queue_size=1, latch=True)
    haf_sub = rospy.Subscriber('/haf_grasping/grasp_hypothesis_with_eval', String, process_haf_grasp)


    rospy.sleep(1)  # LATCHING DONT SEEM TO FIX SO I THINK JUST SLEEP FOR A BIT, MAYBE JUST MAKE LOOK DOWN MOTION OR ACTION INSTEAD
    rospy.loginfo("Moving head down")
    jt = JointTrajectory()
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, -0.98]
    jtp.time_from_start = rospy.Duration(2)
    jt.points.append(jtp)
    head_cmd.publish(jt)

    rospy.loginfo("Moving torso up/down")
    jt = JointTrajectory()
    jt.joint_names = ['torso_lift_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0] #[0.34]
    jtp.time_from_start = rospy.Duration(2)
    jt.points.append(jtp)
    torso_cmd.publish(jt)

    rospy.sleep(2.5)  # SLEEP TO WAIT TIL LOOK DOWN ACTION DONE
    rospy.loginfo("Done")

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("MApart")
    scene.remove_attached_object()  # removes all previously attached objects


    # FILTER, SEGMENT AND CLUSTER TO FIND 3D OBJECT BOUNDING BOX
    rospy.wait_for_service('find_pick')
    try:
        pick_as = rospy.ServiceProxy('find_pick', GetTargetPose)
        resp1 = pick_as()
        rospy.loginfo("Got target pose: " + str(resp1.target_pose))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


    # COMPUTE GRASP USING HAF GRASPING
    rospy.wait_for_service('/haf_grasping/set_grasp_center')
    try:
        set_center_service = rospy.ServiceProxy('/haf_grasping/set_grasp_center', GraspSearchCenter)
        ##p = Point(resp1.target_pose.position)
        resp2 = set_center_service(resp1.target_pose.position)
        if resp2:
            rospy.loginfo("Set search center")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    msg = String()
    msg.data = rospack.get_path('cleanup') + "/data/object/detected_object.pcd"
    haf_pub.publish(msg)


    length = abs(resp1.max_pt.x - resp1.min_pt.x)
    width = abs(resp1.max_pt.y - resp1.min_pt.y)
    height = abs(resp1.max_pt.z - resp1.min_pt.z)
    print length, width, height
    print resp1.min_pt.x, resp1.min_pt.y, resp1.min_pt.z
    print resp1.max_pt.x, resp1.max_pt.y, resp1.max_pt.z

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
    p.pose.position.x = resp1.min_pt.x + length / 2.0
    p.pose.position.y = resp1.min_pt.y + width / 2.0
    p.pose.position.z =  resp1.target_pose.position.z / 2.0# resp1.min_pt.z + height / 2.0#
    scene.add_box("MApart", p, (length, width, resp1.target_pose.position.z))#resp1.target_pose.position.z))

    rospy.sleep(1)
    rospy.loginfo("added objects")

    # define a list of grasps, for now a single grasp
    grasps = [Grasp()]

    grasps[0].id = "test"

    # set the pose of the Pose of the end effector in which it should attempt grasping
    grasps[0].grasp_pose = PoseStamped()
    grasps[0].grasp_pose.header.frame_id = "base_footprint"
    #q = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
    #print "GOING TO", p.pose.position.x, p.pose.position.y, p.pose.position.z# + 0.23
    #grasps[0].grasp_pose.pose = Pose(Point(p.pose.position.x, p.pose.position.y, 0.24), Quaternion(*q))
    while haf_ready == False:
        print "waiting for haf grasp"
        rospy.sleep(5)

    if current_haf_grasp["val"] == -20:
        print "NO GRASP FOUND BY HAF, using basic grasp"
        q = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
        print "GOING TO", p.pose.position.x, p.pose.position.y, p.pose.position.z# + 0.23
        grasps[0].grasp_pose.pose = Pose(Point(p.pose.position.x, p.pose.position.y, 0.24), Quaternion(*q))
    else:
        print "Using haf grasp"
        grasps[0].grasp_pose.pose.position.x, grasps[0].grasp_pose.pose.position.y, grasps[0].grasp_pose.pose.position.z = current_haf_grasp["gcp"]
        grasps[0].grasp_pose.pose.position.z = 0.23  ## this will make almost touch floor # this will make tip of gripper align with position
        q = tf.transformations.quaternion_from_euler(0, 1.57, -math.radians(current_haf_grasp["roll"]))
        grasps[0].grasp_pose.pose.orientation = Quaternion(*q)
        print "hey"
        print grasps[0].grasp_pose.pose
    haf_ready = False
    rospy.sleep(2)

    # define the pre-grasp approach, this is used to define the direction from which to approach the object and the distance to travel
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint"
    grasps[0].pre_grasp_approach.direction.vector.x = 0.0
    grasps[0].pre_grasp_approach.direction.vector.y = 0.0
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0
    grasps[0].pre_grasp_approach.min_distance = 0.1
    grasps[0].pre_grasp_approach.desired_distance = 0.5

    # define the pre-grasp posture, this defines the trajectory position of the joints in the end effector group before we go in for the grasp
    # WE WANT TO OPEN THE GRIPPER, YOU CAN TAKE SOME OF THIS AND MAKE INTO CLOSE GRIPPER FUNCTION
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "arm_tool_link"
    pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    jtpoint = JointTrajectoryPoint()
    #dist = sum([(current_haf_grasp["gp1"][i] - current_haf_grasp["gp2"][i])**2 for i in range(3)]) ** 0.5
    #jtpoint.positions = [dist/2.0, dist/2.0]
    jtpoint.positions = [0.04, 0.04]
    jtpoint.time_from_start = rospy.Duration(2.0)
    pre_grasp_posture.points.append(jtpoint)
    grasps[0].pre_grasp_posture = pre_grasp_posture

    # set the grasp posture, this defines the trajectory position of the joints in the end effector group for grasping the object
    # WE WANT TO CLOSE THE GRIPPER
    grasp_posture = copy.deepcopy(pre_grasp_posture)
    grasp_posture.points[0].time_from_start = rospy.Duration(2.0 + 1.0)
    jtpoint2 = JointTrajectoryPoint()
    #dist = sum([(current_haf_grasp["gp1"][i] - current_haf_grasp["gp2"][i])**2 for i in range(3)]) ** 0.5
    ##print "finger dist = ", str(dist)
    #finger_dist = dist/2.0 # 0.004#0.003
    jtpoint2.positions = [finger_dist, finger_dist]
    jtpoint2.time_from_start = rospy.Duration(2.0 + 1.0 + 3.0)
    jtpoint2.effort.append(1.0)
    jtpoint2.effort.append(1.0)
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
    grasps[0].max_contact_force = 0
    print "max contact force", grasps[0].max_contact_force

    # pick the object
    right_arm.pick("MApart", grasps)


    rospy.loginfo("Moving head back up")
    jt = JointTrajectory()
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, 0.0]
    jtp.time_from_start = rospy.Duration(2.0)
    jt.points.append(jtp)
    head_cmd.publish(jt)
    rospy.sleep(2.5)  # SLEEP TO WAIT TIL LOOK DOWN ACTION DONE
    rospy.loginfo("Done")

    rospy.spin()
    roscpp_shutdown()





if __name__=='__main__' and False:

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

    rospy.loginfo("Setting publishers to torso and head controller...")
    torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1, latch=True)
    head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1, latch=True)
    rospy.sleep(1)  # LATCHING DONT SEEM TO FIX SO I THINK JUST SLEEP FOR A BIT, MAYBE JUST MAKE LOOK DOWN MOTION OR ACTION INSTEAD
    rospy.loginfo("Moving head down")
    jt = JointTrajectory()
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, -0.98]
    jtp.time_from_start = rospy.Duration(2)
    jt.points.append(jtp)
    head_cmd.publish(jt)

    rospy.loginfo("Moving torso up/down")
    jt = JointTrajectory()
    jt.joint_names = ['torso_lift_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0] #[0.34]
    jtp.time_from_start = rospy.Duration(2)
    jt.points.append(jtp)
    torso_cmd.publish(jt)

    rospy.sleep(2.5)  # SLEEP TO WAIT TIL LOOK DOWN ACTION DONE
    rospy.loginfo("Done")

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("MApart")
    scene.remove_attached_object()  # removes all previously attached objects


    # FILTER, SEGMENT AND CLUSTER TO FIND 3D OBJECT BOUNDING BOX
    rospy.wait_for_service('find_pick')
    try:
        pick_as = rospy.ServiceProxy('find_pick', GetTargetPose)
        resp1 = pick_as()
        rospy.loginfo("Got target pose: " + str(resp1.target_pose))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    length = abs(resp1.max_pt.x - resp1.min_pt.x)
    width = abs(resp1.max_pt.y - resp1.min_pt.y) # THIS NEEDS TO FIXED
    height = abs(resp1.max_pt.z - resp1.min_pt.z)
    print length, width, height
    print resp1.min_pt.x, resp1.min_pt.y, resp1.min_pt.z
    print resp1.max_pt.x, resp1.max_pt.y, resp1.max_pt.z
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
    p.pose.position.x = resp1.min_pt.x + length / 2.0
    p.pose.position.y = resp1.min_pt.y + width / 2.0
    p.pose.position.z =  resp1.target_pose.position.z / 2.0# resp1.min_pt.z + height / 2.0#
    scene.add_box("MApart", p, (length, width, resp1.target_pose.position.z))#resp1.target_pose.position.z))

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
    print "GOING TO", p.pose.position.x, p.pose.position.y, p.pose.position.z# + 0.23
    grasps[0].grasp_pose.pose = Pose(Point(p.pose.position.x, p.pose.position.y, 0.24), Quaternion(*q))
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
    grasps[0].pre_grasp_approach.min_distance = 0.1
    grasps[0].pre_grasp_approach.desired_distance = 0.5

    # define the pre-grasp posture, this defines the trajectory position of the joints in the end effector group before we go in for the grasp
    # WE WANT TO OPEN THE GRIPPER, YOU CAN TAKE SOME OF THIS AND MAKE INTO CLOSE GRIPPER FUNCTION
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "arm_tool_link"
    pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = [0.04, 0.04]
    jtpoint.time_from_start = rospy.Duration(2.0)
    pre_grasp_posture.points.append(jtpoint)
    grasps[0].pre_grasp_posture = pre_grasp_posture

    # set the grasp posture, this defines the trajectory position of the joints in the end effector group for grasping the object
    # WE WANT TO CLOSE THE GRIPPER
    grasp_posture = copy.deepcopy(pre_grasp_posture)
    grasp_posture.points[0].time_from_start = rospy.Duration(2.0 + 1.0)
    jtpoint2 = JointTrajectoryPoint()
    finger_dist = (length/2.0) - 0.008
    jtpoint2.positions = [finger_dist, finger_dist]
    jtpoint2.time_from_start = rospy.Duration(2.0 + 1.0 + 3.0)
    jtpoint2.effort.append(1.0)
    jtpoint2.effort.append(1.0)
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
    grasps[0].max_contact_force = 0
    print "max contact force", grasps[0].max_contact_force

    # append the grasp to the list of grasps
    ##grasps.append(g)

    #rospy.sleep(2)

    # pick the object
    right_arm.pick("MApart", grasps)


    rospy.loginfo("Moving head back up")
    jt = JointTrajectory()
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, 0.0]
    jtp.time_from_start = rospy.Duration(2.0)
    jt.points.append(jtp)
    head_cmd.publish(jt)
    rospy.sleep(2.5)  # SLEEP TO WAIT TIL LOOK DOWN ACTION DONE
    rospy.loginfo("Done")

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

    rospy.loginfo("Moving head down")
    jt = JointTrajectory()
    jt.joint_names = ['head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.98]
    jtp.time_from_start = rospy.Duration(2.0)
    jt.points.append(jtp)
    self.head_cmd.publish(jt)
    rospy.loginfo("Done moving head down.")

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
