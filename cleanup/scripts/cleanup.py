#!/usr/bin/env python
# Author: Jason De Souza <k1922008@kcl.ac.uk>
from __future__ import print_function
import rospy



from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from std_srvs.srv import Empty, EmptyRequest
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

class ManipulationArea:

    def __init__(self, name, position, orientation):

        self.name = name
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1, latch=True)

        self.move_base_client = SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("...connected to move base client")

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

        self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)


        ### another option for orientation:
        #quaternionArray = tf.transformations.quaternion_about_axis(theta, (0,0,1))
		# quaternion_about_axis offers a convenient way for calculating the members of a quaternion.
		# In order to use it we need to convert it to a Quaternion message structure
		#moveBaseGoal.target_pose.pose.orientation = self.array_to_quaternion(quaternionArray)

        ###self.go()

    def go(self, pickup=False):
        if pickup:
            # do pickup pose
            pass

        self.raise_head()
        print("moving to " + self.name)
        # use move base to go
        self.move_base_client.send_goal(self.goal)
        self.move_base_client.wait_for_result()
        print("i think it's reached the destination, proceeding with inspect surroungins")
        self.clear_octomap_srv.call(EmptyRequest())
        self.inspect_surroundings()  # to build octomap
        rospy.loginfo("inspected surroundings, waiting for next task at " + self.name)

    def inspect_surroundings(self):
        client = SimpleActionClient("play_motion", PlayMotionAction)
        client.wait_for_server()
        rospy.loginfo("...connected to play_motion client")
        goal = PlayMotionGoal()#
        goal.motion_name = 'inspect_surroundings'# CHANGE TO 'inspect_surroundings'
        goal.skip_planning = True
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(10.0))

    def raise_head(self):
        rospy.loginfo("Making head look directly forward")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, 0.0]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")



class Drawer(ManipulationArea):

    def __init__(self, name, position, orientation):
        ##super.__init__(name, position, orientation)  #  python 3
        super(Drawer, self).__init__(name, position, orientation)  #  python 2

        ## CANT OPEN OR CLOSE WHILE HOLDING SOMETHING, WHAT TO DO??
    def go(self, pickup=False):
        # move in front of drawer
        super(Drawer, self).go()
        # detect pose of drawer handle
        ###
        # generate way points to open drawer
        ###
        # follow waypoints to open drawer

    def open(self):
        pass

    def close(self):
        pass

    ## def generate_place_areas/place_area constraints

    def get_place_locations(self):  # SHOULD WE ADD FUNCTION LIKE THIS?
        pass

class Container(ManipulationArea):

    def __init__(self, name, position, orientation):
        super(Drawer, self).__init__(name, position, orientation)

    def generate_place_areas(self):
        ## do pose estimation to find container

        ## computer point above center of container

        pass


    def get_place_locations(self):  # SHOULD WE ADD FUNCTION LIKE THIS?
        pass





import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from yolo4.srv import DetectObjects
# for 3d point to pixel
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker
## the attributes are cam and trans
class ClusterLabeler:

    def __init__(self):
        # setup tf stuff
        self.tf_buffer = tf2_ros.Buffer()
        # NOTE: this must be created some time before you use it or you could get frame not exist warning
        # source for NOTE: https://answers.ros.org/question/192570/tftransformlistenertransformpose-exception-target_frame-does-not-exist/
        #create a tf2_ros.TransformListener object. Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.
        listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_pub = rospy.Publisher('clusters_text_markers', MarkerArray, queue_size=5)



        # setup camera
        camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
        print(camera_info.width)
        print(camera_info.height)

        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(camera_info)
        print(self.cam.tfFrame())




    def label_clusters(self, clusters):
        detections = self.get_yolo_detections()

        ## NEED TO GO FROM CLUSTERS/POINT CLOUDS TO IMAGE POINTS

        # get transformation so you can convert pointclouds (which are in base_footprint) to xtion_rgb_optical_frame
        trans = None
        try:
            print("HI THERE GONNA TRY AND LOOKUP A TRANSFROM")
            trans = self.tf_buffer.lookup_transform("xtion_rgb_optical_frame","base_footprint", rospy.Time())
            print("DID THE TRANSFORM LOOKUP")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(ex)

        print("detections: ")
        for detection in detections:
            print(detection.label)

        text_marker_data = []
        for cluster in clusters:

            # transform to camera frame
            transformed_pointcloud = do_transform_cloud(cluster.pointcloud, trans)

            # get pixels by computing projection
            pixels = set()
            cloud_points = list(point_cloud2.read_points(transformed_pointcloud, skip_nans=True, field_names = ("x", "y", "z")))
            for point in cloud_points:
                pixels.add(self.cam.project3dToPixel((point[0], point[1], point[2])))


            # go through each yolo and find the one with max overlay
            max_percentage_filled = 0
            max_label = "unknown"
            # MAYBE ADD A THRESHOLD TOO? i.e percentage of points in detection box
            for detection in detections:
                current_overlap = 0
                for pixel in pixels:
                    if pixel[0] >= detection.x and pixel[0] <= detection.x + detection.width and pixel[1] >= detection.y and pixel[1] < detection.y + detection.height:
                        current_overlap += 1

                percentage_filled = (1.0 * current_overlap) / (detection.width * detection.height)
                if percentage_filled > max_percentage_filled:
                    max_percentage_filled = percentage_filled
                    max_label = detection.label
            print(max_label, " picked as fill: ", max_percentage_filled)


            #cloud_points_base_frame = list(point_cloud2.read_points(cluster.pointcloud, skip_nans=True, field_names = ("x", "y", "z")))
            text_marker_data.append([max_label, cluster.top_center_pose.position.x, cluster.top_center_pose.position.y, cluster.max_pt.z + 0.1])


        ## IF YOU PREFER MAYBE MAKE MARKER CLASS LIKE THE CONSTRUCT, BUT I THINK THIS IS OKAY
        self.publish_text_markers(text_marker_data, 10)  # display text markers above each cluster for 10 seconds
        # return list of labels
        return [data[0] for data in text_marker_data]


    def get_yolo_detections(self):
        # call yolo service to get boxes
        rospy.wait_for_service('/yolo_server/detect_objects')
        try:
            print("hey 2.1")
            get_detections_service = rospy.ServiceProxy('/yolo_server/detect_objects', DetectObjects)
            ##p = Point(resp1.target_pose.position)
            print("hey 2.2")

            get_detections_response = get_detections_service()

            if get_detections_response:
                rospy.loginfo("obtained yolo detections")
                print("Got ", len(get_detections_response.detections))
                print("hey 2.3")

        except rospy.ServiceException as e:
            print("DetectObjects service call failed: %s"%e)

        return get_detections_response.detections

    def generate_text_markers(self, text_marker_data, timeout_duration):
        marker_idx = 0
        marker_array_msg = MarkerArray()
        for i in range(len(text_marker_data)):
            marker = Marker()
            marker.header.frame_id = "base_footprint" # MIGHT WANNA CHANGE TO MAP, if you do change to map you probs have to transform the pose
            marker.id = i
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.text = text_marker_data[i][0]
            ##marker.pose = Pose()
            marker.pose.position.x = text_marker_data[i][1]
            marker.pose.position.y = text_marker_data[i][2]
            marker.pose.position.z = text_marker_data[i][3]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.lifetime = rospy.Duration(timeout_duration)
        # Set the namespace and id for this marker.  This serves to create a unique ID
        # Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "cluster_label"
            marker_array_msg.markers.append(marker)
        return marker_array_msg


    def publish_text_markers(self, text_marker_data, timeout_duration=5):
        # publish just once: source: https://answers.ros.org/question/267199/how-can-i-publish-exactly-once-when-the-node-is-run/
        try:
            rate = rospy.Rate(1)  # 10hz
            while not rospy.is_shutdown():
                connections = self.marker_pub.get_num_connections()
                rospy.loginfo('Connections: %d', connections)
                if connections > 0:
                    self.marker_pub.publish(self.generate_text_markers(text_marker_data, timeout_duration))
                    rospy.loginfo('Published')
                    break
                    rate.sleep()
        except rospy.ROSInterruptException, e:
            raise e

        print("DONE with publishing cluster markers")







# PROBS BETTER TO MAKER THIS ONE A SERVICE/SERVER NO???, LIKE THE TIAGO TUTORIALS, IT WILL STILL BE ABLE TO MAINTIAN STATE
# NEED TO PASS IN CLUSTER
# SERVICES AVAILBALE ARE PICK, PLACE
# CAN PROBS EASILY SWITCH INTO TURN INTO SERVER/SERVICE LATER IF YOU LAY OUT THIS CLASS CAREFULLY
# (or could define as action server if you want feedback, use matteos tutorial to learn about action server creation)

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from sensor_msgs import point_cloud2
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from sensor_msgs.msg import PointCloud2
from haf_grasping.srv import *  ## still needed ??
from haf_grasping.msg import GraspInput#, GraspOutput
from geometry_msgs.msg import PoseStamped, Quaternion
from haf_grasping.msg import CalcGraspPointsServerAction, CalcGraspPointsServerGoal
from moveit_msgs.msg import Grasp, PlaceLocation#, GripperTranslation,
import tf  # just for transformations, if you're doing stuff like broadcasting then use tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import copy
from moveit_commander import MoveGroupCommander, RobotCommander
import sys
from std_srvs.srv import Empty, EmptyRequest
##from geometry_msgs.msg import , Point,Pose, PoseStamped, PoseArray,
import os

# make global for now
#plane_max_pt = None
#plane_min_pt = None


# prepare_robot, raise_arm, lift_torso, lower_head are from tiago_tutorials: tiago_pick_demo, pick_client.py
class Manipulator:

    def __init__(self):

        #roscpp_initialize(sys.argv)  # for moveit stuff


        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.play_motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_motion_client.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.arm_torso = MoveGroupCommander("arm_torso")
        print(self.arm_torso.get_goal_joint_tolerance())
        print(self.arm_torso.get_current_joint_values())
        print(self.arm_torso.get_named_targets())
        self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected!")

        #print "============ Robot Groups:"
        print(self.robot.get_group_names())
        rospy.loginfo("Creating publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1, latch=True)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1, latch=True)
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing Manipulator.")


        self.plane_min_pt = None
        self.plane_max_pt = None



    def pick(self, cluster, label):

        self.clear_scene()
        self.lower_head()
        #self.unfold_arm()
        self.prepare_robot()
        self.lift_torso()
        rospy.sleep(2.0)  # Removing is fast

        # add object/s to planning scene (one to be grasped and maybe table if octomap not enough)
        p = PoseStamped()
        p.header.frame_id = "base_footprint"#robot.get_planning_frame()
        # add an object to be grasped
        pose_est, file_name = self.get_pose_estimate(cluster, label)
        use_mesh = False
        if file_name and os.path.exists("/home/jason/Desktop/ycb_models/ycb-tools/models/ycb/" + file_name + "/google_16k/nontextured.stl"):
            use_mesh = True
            p.pose = pose_est
            # add mesh from ycb dataset
            rospy.loginfo("closest match for " + label + " was " + file_name)
            self.scene.add_mesh("object_to_pick", p, "/home/jason/Desktop/ycb_models/ycb-tools/models/ycb/" + file_name + "/google_16k/nontextured.stl")
        else:
            # estimate with 3d bounding box
            p.pose.position.x = cluster.min_pt.x + ((cluster.max_pt.x - cluster.min_pt.x) / 2.0)
            p.pose.position.y = cluster.min_pt.y + ((cluster.max_pt.y - cluster.min_pt.y) / 2.0)
            p.pose.position.z =  cluster.min_pt.z + ((cluster.max_pt.z - cluster.min_pt.z) / 2.0)#cluster.target_pose.position.z / 2.0# resp1.min_pt.z + height / 2.0#
            self.scene.add_box("object_to_pick", p, ((cluster.max_pt.x - cluster.min_pt.x)+0.025, (cluster.max_pt.y - cluster.min_pt.y)+0.025, 2*(cluster.max_pt.z - cluster.min_pt.z)))#resp1.target_pose.position.z))

        # add table
        p.pose.orientation = Quaternion(0,0,0,1)
        p.pose.position.x = self.plane_min_pt.x + (self.plane_max_pt.x - self.plane_min_pt.x) / 2.0
        p.pose.position.y = self.plane_min_pt.y + (self.plane_max_pt.y - self.plane_min_pt.y) / 2.0
        p.pose.position.z = self.plane_min_pt.z - 0.1
        self.scene.add_box("table", p, ((self.plane_max_pt.x - self.plane_min_pt.x), (self.plane_max_pt.y - self.plane_min_pt.y), (0.001)))#resp1.target_pose.position.z))
        self.arm_torso.set_support_surface_name("table")

        rospy.sleep(1)
        rospy.loginfo("added objects")

        # set search center for haf grasping and get grasp
        grasp = self.get_haf_grasp(cluster)

        # define a list of grasps, for now a single grasp
        grasps = [Grasp()]


        grasps[0].id = "top_down"

        # set the pose of the Pose of the end effector in which it should attempt grasping
        grasps[0].grasp_pose = PoseStamped()
        grasps[0].grasp_pose.header.frame_id = "base_footprint"

        if grasp.graspOutput.eval == -20:
            rospy.logwarn("NO GRASP FOUND BY HAF")
            self.clear_scene()
            return False
        else:
            print( "Using haf grasp")
            grasps[0].grasp_pose.pose.position = grasp.graspOutput.averagedGraspPoint
            # distance from tool link to end of gripper is 0.23m
            tool_center_point_offset = 0.03  # 3cm from tip of gripper
            if (grasp.graspOutput.averagedGraspPoint.z - self.plane_max_pt.z) >= tool_center_point_offset:
                # use ideal grasp
                rospy.loginfo("Using ideal grasp")
                grasps[0].grasp_pose.pose.position.z += (0.23 - tool_center_point_offset)
            else:
                # get close to table when grasping
                rospy.loginfo("Grasp pose of tip using tool center point would go through table, so setting tip of gripper to just above table")
                grasps[0].grasp_pose.pose.position.z = self.plane_max_pt.z + 0.23

            q = tf.transformations.quaternion_from_euler(0, 1.57, -grasp.graspOutput.roll)
            grasps[0].grasp_pose.pose.orientation = Quaternion(*q)
            print( "hey")
            print( grasps[0].grasp_pose.pose)
        #rospy.sleep(2)

        # define the pre-grasp approach, this is used to define the direction from which to approach the object and the distance to travel
        grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint"
        # THIS MAYBE
        #grasps[0].pre_grasp_approach.direction.vector = grasp.graspOutput.approachVector
        grasps[0].pre_grasp_approach.direction.vector.x = 0.0
        grasps[0].pre_grasp_approach.direction.vector.y = 0.0
        grasps[0].pre_grasp_approach.direction.vector.z = -1.0
        grasps[0].pre_grasp_approach.min_distance = 0.05#0.1
        grasps[0].pre_grasp_approach.desired_distance = 0.5

        self.open_pre_grasp(grasps[0])
        self.close_grasp_posture(grasps[0])

        # set the post-grasp retreat, this is used to define the direction in which to move once the object is grasped and the distance to travel.
        grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint"
        grasps[0].post_grasp_retreat.direction.vector.x = 0.0
        grasps[0].post_grasp_retreat.direction.vector.y = 0.0
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0
        grasps[0].post_grasp_retreat.desired_distance = 0.25
        grasps[0].post_grasp_retreat.min_distance = 0.01

        #Don't restrict contact force
        grasps[0].max_contact_force = 0.0
        print( "max contact force", grasps[0].max_contact_force)


        print("the grasp: ", grasps[0])
        rospy.sleep(5)


        # pick the object
        self.arm_torso.pick("object_to_pick", grasps)
        if use_mesh:
            self.scene.attach_mesh("arm_tool_link", "object_to_pick", touch_links=self.robot.get_link_names(group="gripper"))  # attach object manually in case goal tolerance violated
        else:
            self.scene.attach_box("arm_tool_link", "object_to_pick", touch_links=self.robot.get_link_names(group="gripper"))  # attach object manually in case goal tolerance violated

        self.scene.remove_world_object("table")
        self.lift_torso()  ## SHOULD PROBS RAISE ARM COZ WHAT IF ALREADY LIFTED?
        self.home_position()
        self.raise_head()

        # check effort to see if object has been grasped (effor is -16 if grasped)
        from sensor_msgs.msg import JointState
        joint_info = rospy.wait_for_message("/joint_states", JointState)
        rospy.loginfo(joint_info.effort[joint_info.name.index("gripper_left_finger_joint")])
        rospy.loginfo(joint_info.effort[joint_info.name.index("gripper_right_finger_joint")])

        if joint_info.effort[joint_info.name.index("gripper_left_finger_joint")] <= -15 and joint_info.effort[joint_info.name.index("gripper_right_finger_joint")] <= -15:
            rospy.loginfo("OBJECT SUCCESSFULLY GRASPED")
        else:
            rospy.logwarn("OBJECT NOT GRASPED PROPERLY")
            self.clear_scene()
            return False


        #roscpp_shutdown()# for moveit stuff

        return True


    # make gripper open during pre grasp stage
    def open_pre_grasp(self, grasp):
        pre_grasp_posture = JointTrajectory()
        pre_grasp_posture.header.frame_id = "arm_tool_link"
        pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [0.04, 0.04]
        jtpoint.time_from_start = rospy.Duration(2.0)
        pre_grasp_posture.points.append(jtpoint)
        grasp.pre_grasp_posture = pre_grasp_posture


    # make gripper close during grasping stage
    def close_grasp_posture(self, grasp):
        ##grasp_posture = copy.deepcopy(pre_grasp_posture)
        grasp_posture = JointTrajectory()
        grasp_posture.header.frame_id = "arm_tool_link"
        grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        #grasp_posture.points[0].time_from_start = rospy.Duration(2.0 + 1.0)
        jtpoint = JointTrajectoryPoint()
        finger_dist = 0.0002  # goal joint tolerance is 0.0001
        print( "finger dist = ", str(finger_dist))
        jtpoint.positions = [finger_dist, finger_dist]
        jtpoint.time_from_start = rospy.Duration(2.0 + 1.0 + 3.0)
        grasp_posture.points.append(jtpoint)
        grasp.grasp_posture = grasp_posture



    def place(self, obstacle_clusters):

        self.prepare_robot()
        self.lift_torso()

        # THIS PROBS WON'T WORK FOR MESH, SO MAYBE JUST DON'T USE MESH FOR ALL ITEMS EXCEPT CUTLERY
        # THE ITEMS THAT USE MESH WILL BE DIFF AS THEY NEED REORIENTATION AND NEED TO BE PLACED IN CONTAINER, SO MAKE ENTIRELY DIFFERENT FUNCTINO IF YOU WANT
        # places on a flat surface
        # MAYBE COULD ALSO CREATE A PLACE_CONTAINER function, that gets the cluster and places in center of it

        place_locations = []
        resolution = 10.0  # 10cm
        place_height_padding = 0.01  # the height the bottom of object should be above plane before being released from gripper
        ## get height from attached object
        l, w, h = self.scene.get_attached_objects()["object_to_pick"].object.primitives[0].dimensions


        padding = max(0.1, max(l, w))  # ensure object is at least 10cm from edge of table
        start_x = self.plane_min_pt.x + padding
        end_x = self.plane_max_pt.x - padding
        start_y = self.plane_min_pt.y + padding
        end_y = self.plane_max_pt.y - padding
        # this is the distance from the arm tool link to the base of the object
        # the get_attached_objects function returns information about the attached object in the the arm_tool_link frame
        # the x axis of the tool_link frame is aligned with the gripper
        link_object_bottom_dist = self.scene.get_attached_objects()["object_to_pick"].object.primitive_poses[0].position.x + (h / 2.0)

        print(self.scene.get_attached_objects()["object_to_pick"].object.primitive_poses[0].position)
        rospy.loginfo("object lwh: " + str(l) + " " + str(w) + " " + str(h))
        rospy.loginfo("distance from arm tool link to base of object" + str(link_object_bottom_dist))

        rospy.loginfo("")
        z = self.plane_max_pt.z + link_object_bottom_dist + place_height_padding
        resolution = 0.1  # 10cm

        current_x = start_x
        while current_x <= end_x:
            current_y = start_y
            while current_y <= end_y:
                place_locations.append(self.create_place_location(current_x, current_y, z))
                current_y += resolution
            current_x += resolution



        rospy.loginfo("Placing object")
        self.arm_torso.set_planning_time(10)
        self.arm_torso.place("object_to_pick", place_locations)
        rospy.loginfo("Done placing")
        self.home_position()
        self.clear_scene()

    def create_place_location(self, x, y, z):
        # creates a top down release place location
        rospy.loginfo("Generating place location")

        place_location = PlaceLocation()
        # set place location pose
        place_location.place_pose = PoseStamped()
        place_location.place_pose.header.frame_id = "base_footprint"
        #q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        place_location.place_pose.pose.position.x = x
        place_location.place_pose.pose.position.y = y
        place_location.place_pose.pose.position.z = z
        place_location.place_pose.pose.orientation.w = 1

        rospy.loginfo("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

        # set pre place approach
        place_location.pre_place_approach.direction.header.frame_id = "base_footprint"
        # set direction to negative z axis (move arm down)
        place_location.pre_place_approach.direction.vector.z = -1.0
        place_location.pre_place_approach.min_distance = 0.0000095
        place_location.pre_place_approach.desired_distance = 0.115

        # set post grasp retreat
        place_location.post_place_retreat.direction.header.frame_id = "base_footprint"
        # set direction to oositive z axis (move arm up)
        place_location.post_place_retreat.direction.vector.z = 1.0
        place_location.post_place_retreat.min_distance = 0.00001
        place_location.post_place_retreat.desired_distance = 0.25

        # set posture of eef after placing object (open gripper)
        post_place_posture = JointTrajectory()
        post_place_posture.header.frame_id = "arm_tool_link"
        post_place_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [0.04, 0.04]
        jtpoint.time_from_start = rospy.Duration(2.0)
        post_place_posture.points.append(jtpoint)
        place_location.post_place_posture = post_place_posture

        return place_location








    def place_bin(self):  # COULD USE THIS TO PLACE IN BIN MANYALLY

        ##self.clear_octomap_srv.call(EmptyRequest())
        #self.unfold_arm()
        #rospy.sleep(3)

        rospy.loginfo("Generating place locations")

        place_locations = []

        place_location = PlaceLocation()
        # set place location pose
        place_location.place_pose = PoseStamped()
        place_location.place_pose.header.frame_id = "base_footprint"
        #q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        place_location.place_pose.pose.position.x = 0.8
        place_location.place_pose.pose.position.y = 0.0
        place_location.place_pose.pose.position.z = 0.4
        #place_location.place_pose.pose.position.x = 0.5
        #place_location.place_pose.pose.orientation = Quaternion(*q)
        place_location.place_pose.pose.orientation.w = 1

        # set pre place approach
        place_location.pre_place_approach.direction.header.frame_id = "base_footprint"
        # set direction to negative z axis (move arm down)
        place_location.pre_place_approach.direction.vector.z = -1.0
        place_location.pre_place_approach.min_distance = 0.0000095
        place_location.pre_place_approach.desired_distance = 0.01

        # set post grasp retreat
        place_location.post_place_retreat.direction.header.frame_id = "base_footprint"
        # set direction to oositive z axis (move arm up)
        place_location.post_place_retreat.direction.vector.z = 1.0
        place_location.post_place_retreat.min_distance = 0.00001
        place_location.post_place_retreat.desired_distance = 0.2

        # set posture of eef after placing object (open gripper)
        post_place_posture = JointTrajectory()
        post_place_posture.header.frame_id = "arm_tool_link"
        post_place_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [0.04, 0.04]
        jtpoint.time_from_start = rospy.Duration(2.0)
        post_place_posture.points.append(jtpoint)
        place_location.post_place_posture = post_place_posture

        print("PLACE LOCATION")
        print(place_location)
        place_locations.append(place_location)

        rospy.loginfo("Placing object")
        self.arm_torso.set_planning_time(10)
        self.arm_torso.place("object_to_pick", place_locations)
        rospy.loginfo("Done placing")

        self.home_position()
        self.clear_scene()

        #self.scene.remove_attached_object()  # removes all previously attached objects


    def clear_scene(self):
        self.scene.remove_attached_object()
        self.scene.remove_world_object()
        self.scene.clear()

    def get_haf_grasp(self, cluster):

        # should we actually pass in full point cloud (get from robot, can match timestamp if you want/need), so you can avoid other stuff,
        #  .. but if you do woudl it even try to avoid other stuff or even try to grab them

        client = SimpleActionClient('calc_grasppoints_svm_action_server', CalcGraspPointsServerAction)
        client.wait_for_server()
        goal = CalcGraspPointsServerGoal()
        print(dir(goal.graspinput))
        goal.graspinput.input_pc = cluster.pointcloud

        goal.graspinput.goal_frame_id = "base_footprint"
        #from geometry_msgs.msg import Point
        goal.graspinput.grasp_area_center = cluster.top_center_pose.position##Point(cluster.min_pt.x + (cluster.max_pt.x - cluster.min_pt.x)/2.0, cluster.min_pt.y + (cluster.max_pt.y - cluster.min_pt.y)/2.0, cluster.max_pt.z/2.0)#cluster.top_center_pose.position

        # forward grasping,could also just change change approach vec and it might work, though you kinda need whole pointcloud.
        # TRUTHFULLY, IT MIGHT BE BETTER IF YOU STICK WITH TOP DOWN TO GET GRASP POINT, THEN SEND GRIPPER TO THAT POSITION, BUT GRIPPER CAN BE IN ANY ORIENTATION
        #goal.graspinput.grasp_area_center.x  = cluster.min_pt.x
        #goal.graspinput.grasp_area_center.y  = cluster.top_center_pose.position.y
        #goal.graspinput.grasp_area_center.z  = cluster.min_pt.z + ((cluster.max_pt.z - cluster.min_pt.z)/2.0)


        goal.graspinput.grasp_area_length_x = 32
        goal.graspinput.grasp_area_length_y = 44
        goal.graspinput.max_calculation_time = rospy.Duration(40)
        goal.graspinput.show_only_best_grasp = False
        goal.graspinput.threshold_grasp_evaluation = False
        goal.graspinput.approach_vector.x = 0
        goal.graspinput.approach_vector.y = 0
        goal.graspinput.approach_vector.z = 1
        goal.graspinput.gripper_opening_width = 1
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(40))
        grasp = client.get_result()
        # client.get_state()??
        print(grasp)
        return grasp


    # delete me this is jsut a test
    def get_drawer_grasp(self, pose):
        grasp = Grasp()
        grasp.grasp_pose.header.frame_id = "base_footprint"
        grasp.grasp_pose.pose = pose
        grasp.grasp_pose.pose.position.x -= 0.23  # this will make tip of gripper align with position


        ## COULD USE HAF GRASP ON THE HANDLE TO ACCOUNT FOR POSE ESTIMATION ERROR, but first see if there is big error when using full pointcloud

        #q = tf.transformations.quaternion_from_euler(0, 1.57, -grasp.graspOutput.roll)
        #grasp.grasp_pose.pose.orientation = Quaternion(*q)

        grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint"
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.direction.vector.y = 0.0
        grasp.pre_grasp_approach.direction.vector.z = 0.0
        grasp.pre_grasp_approach.min_distance = 0.05#0.1
        grasp.pre_grasp_approach.desired_distance = 0.5

        self.open_pre_grasp(grasp)
        self.close_grasp_posture(grasp)

        grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint"
        grasp.post_grasp_retreat.direction.vector.x = -1.0
        grasp.post_grasp_retreat.direction.vector.y = 0.0
        grasp.post_grasp_retreat.direction.vector.z = 0.0
        grasp.post_grasp_retreat.desired_distance = 0.5
        grasp.post_grasp_retreat.min_distance = 0.1


        #Don't restrict contact force
        grasp.max_contact_force = 0.0

        return grasp



    # IDK IF THIS CLASS IS THE BEST PLACE TO PUT THIS
    def get_pose_estimate(self, cluster, label):
        # IF YOU ARE NOTICING ISSUES WITH THIS THEN MAYBE DEEPCOPY THE CLUSTER BEFORE PASSING IT INTO THE SERVICE


        rospy.loginfo("trying to find pose estimate for " + label)
        ## BETTER TO PULL THESE FROM FILE IF POSSIBLE
        ## you may be able to create another file like coco.names and load it instead when using yolo, up to you !!!!!
        ## mappings can include things it commonly gets mistaken for, that way it may corect it and return corrected label
        ##round_objects = ["013_apple","017_orange","053_mini_soccer_ball","054_softball", "055_baseball", "056_tennis_ball", "057_racquetball", "058_golf_ball"]
        yolo_to_ycb_names = {
            "apple": ["013_apple"],
            "banana" : ["011_banana"],
            "orange": ["017_orange"],
            "fork" : ["030_fork"],
            "knife": ["032_knife"],
            "spoon": ["031_spoon"],
            "bowl": ["024_bowl"],
            "cup":["025_mug"],
            "donut": [],
            "sports ball": ["053_mini_soccer_ball", "054_softball", "055_baseball", "056_tennis_ball", "057_racquetball", "058_golf_ball"],
            "scissors":[],
        }

        if label not in yolo_to_ycb_names or len(yolo_to_ycb_names[label]) == 0:
            rospy.loginfo("cannot find pose estimate for an unknown object")
            # PUT THE RETURN NONE BACK AND REMOVE FOLLOWING LINE, THIS IS JUST FOR TESTING
            #label = "fork"
            return None, None


        print("hey 415")
        file_name = None
        from geometry_msgs.msg import Pose
        pose = Pose()
        # call PoseEst service to estimate pose
        rospy.wait_for_service('/get_pose_estimate')
        try:
            print("hey 931")
            get_pose_est_service = rospy.ServiceProxy('/get_pose_estimate', GetPoseEst)
            print("hey 852")
            from std_msgs.msg import String
            get_pose_est_response = get_pose_est_service(cluster.pointcloud, [String(s) for s in yolo_to_ycb_names[label]])
            file_name = yolo_to_ycb_names[label][get_pose_est_response.index]

            # convert from transform msg to pose msg
            pose.position.x = get_pose_est_response.estimated_transform.translation.x
            pose.position.y = get_pose_est_response.estimated_transform.translation.y
            pose.position.z = get_pose_est_response.estimated_transform.translation.z
            pose.orientation = get_pose_est_response.estimated_transform.rotation

            print("hey 825")
            print(get_pose_est_response)
        except rospy.ServiceException as e:
            print("GetClusters service call failed: %s"%e)


        rospy.loginfo("done trying to find pose estimate")
        return pose, file_name
        #return get_clusters_response.clusters

    def set_plane_min_pt(self, point):
        print("plane min point set to ", point)
        self.plane_min_pt = point

    def set_plane_max_pt(self, point):
        print("plane max point set to ", point)
        self.plane_max_pt = point

    def unfold_arm(self):
        rospy.loginfo("unfolding arm")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'unfold_arm'
        pmg.skip_planning = False
        self.play_motion_client.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")

    def home_position(self):
        rospy.loginfo("moving arm to home position")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'home'
        pmg.skip_planning = False
        self.play_motion_client.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")

    def prepare_robot(self):
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_motion_client.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")
        self.lower_head()
        rospy.loginfo("Robot prepared.")

    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        rospy.sleep(2.5)
        self.torso_cmd.publish(jt)

    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.98]#-0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.sleep(2.0)
        rospy.loginfo("Done.")

    def raise_head(self):
        rospy.loginfo("Making head look directly forward")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, 0.0]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.sleep(2.0)
        rospy.loginfo("Done.")

    def raise_arm(self):
        # need to make arm raise above its current position
        pass
        # Raise arm
	#rospy.loginfo("Moving arm to a safe pose")
	#pmg = PlayMotionGoal()
#        pmg.motion_name = 'pick_final_pose'
#	pmg.skip_planning = False
#	self.play_motion_client.send_goal_and_wait(pmg)
#	rospy.loginfo("Raise object done.")


#    def unfold_arm(self):
#        rospy.loginfo("unfolding arm")
#        pmg = PlayMotionGoal()
#        pmg.motion_name = 'unfold_arm'
#        self.play_motion_clien

    def drop(self): # DOES NOT NECESSARYILY NEED TO BE IN THIS CLASS
        pass







import tf2_ros
from pcd_proc.srv import GetClusters, GetPoseEst
class Cleanup:

    def __init__(self, manipulation_areas, pickup_indexes, label_all = False):
        self.manipulation_areas = manipulation_areas
        self.pickup_indexes = pickup_indexes
        self.object_destinations = {"unknown":0, "apple": 0, "banana":0, "fork":0, "sports ball":0, "donut":0}
        # for debugging/visual purpose can label all clusters instead of just the one to be picked
        self.label_all = label_all
        self.cluster_labeler = ClusterLabeler()
        self.manipulator = Manipulator()



    def run_cleanup(self):

        for pickup_index in self.pickup_indexes:
            clean = False
            while not clean:
                self.manipulation_areas[pickup_index].go(True)
                clusters = self.get_clusters()

                # MAYBE DISREGARD/POP ANY CLUSTERS THAT ARE OUT OF REACH or too big?? such as the master chef can, can use top pose though ideally use grasp points 1 and 2 to check

                if len(clusters) == 1:
                    clean = True
                    rospy.loginfo("Cleaning last item at, shouldn't need to visit here " + self.manipulation_areas[pickup_index].name + " again.")
                elif len(clusters) == 0:
                    # should only get here if something else moved an item while the robot was away
                    clean = True
                    rospy.loginfo("Seems there aren't any more items at " + self.manipulation_areas[pickup_index].name + ".")
                    break

                # find which object is closest in x direction (closer to front edge of table)
                closest_cluster_index = self.get_closest(clusters)
                if abs(clusters[closest_cluster_index].top_center_pose.position.y) > 0.1:  # if y difference greater than threshold  # COULD CHANGE TO IF XY EUC DIST IS GREATER THAN THRESH
                    # move to be in front of it and recalculate clusters
                    # we move as reach is circuluar, so it greatest when in front, may also help object detection
                    # we recaculate as amcl has uncertainty (and some object models like banana move by themselves overtime)

                    self.manipulator.raise_head()  # raise head so can move without pointcloud detecting edge of table
                    from actionlib import SimpleActionClient
                    from tf2_geometry_msgs import PointStamped
                    from geometry_msgs.msg import Point

                    move_base_client = SimpleActionClient("/move_base", MoveBaseAction)
                    move_base_client.wait_for_server()
                    rospy.loginfo("...connected to move base client, adjusting robot position to be in front of object")
                    # Define the goal
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    # get robot current position in map
                    # get a point along x axis of robot base frame (i.e x =0), and y= cluster.y
                    ideal_robot_pt = PointStamped()
                    #ideal_robot_pos.header.stamp = rospy.Time.now()
                    ideal_robot_pt.header.frame_id = "base_footprint"
                    ideal_robot_pt.point.x = 0.0
                    ideal_robot_pt.point.y = clusters[closest_cluster_index].top_center_pose.position.y
                    ideal_robot_pt.point.z = 0.0

                    # transform that point to the map frame
                    tf_buffer = tf2_ros.Buffer()
                    listener = tf2_ros.TransformListener(tf_buffer)
                    rospy.sleep(1.0)
                    target_pt = tf_buffer.transform(ideal_robot_pt, "map")

                    print(target_pt)
                    # make robot move to that point
                    goal.target_pose.pose.position.x = target_pt.point.x
                    goal.target_pose.pose.position.y = target_pt.point.y
                    goal.target_pose.pose.position.z = target_pt.point.z
                    goal.target_pose.pose.orientation.x = 0
                    goal.target_pose.pose.orientation.y = 0
                    goal.target_pose.pose.orientation.z = 0
                    goal.target_pose.pose.orientation.w = 1
                    move_base_client.send_goal(goal)
                    move_base_client.wait_for_result()
                    print("Adjusted robot position")

                    self.manipulator.lower_head()  # lower head so can look at objects again


                    clusters = self.get_clusters()
                    # same cluster should be identified, by order of list may have change so recompute
                    closest_cluster_index = self.get_closest(clusters)



                label = ""
                if self.label_all:
                    labels = self.cluster_labeler.label_clusters(clusters)
                    label = labels[closest_cluster_index]
                else:
                    label = self.cluster_labeler.label_clusters([clusters[closest_cluster_index]])[0]


                print("picking object called " + label)
                self.manipulator.clear_scene()
                print("picking cluster number " + str(closest_cluster_index))
                pick_success = self.manipulator.pick(clusters[closest_cluster_index], label)
                if pick_success:
                    destination = self.object_destinations[label]
                    self.manipulation_areas[destination].go(False)
                    place_area_clusters = self.get_clusters()
                    print("placing object called " + label)
                    self.manipulator.place(place_area_clusters)
                else:
                    rospy.logwarn("Unable to pick from this area. Skipping this area.")
                    break

    def get_clusters(self):
        # call cluster service to get clusters
        rospy.wait_for_service('/get_clusters')
        try:
            get_clusters_service = rospy.ServiceProxy('/get_clusters', GetClusters)
            get_clusters_response = get_clusters_service()
            if get_clusters_response:
                rospy.loginfo("Got " + str(len(get_clusters_response.clusters)) + " clusters")
        except rospy.ServiceException as e:
            print("GetClusters service call failed: %s"%e)

        self.manipulator.set_plane_min_pt(get_clusters_response.plane_min_pt)
        self.manipulator.set_plane_max_pt(get_clusters_response.plane_max_pt)

        return get_clusters_response.clusters

    def get_closest(self, clusters):
        # all clusters are in base frame, so closest is just distance of each from origin

        closest_idx = 0
        #closest_dist = clusters[0].top_center_pose.position.x + clusters[0].top_center_pose.position.y
        closest_dist = clusters[0].top_center_pose.position.x
        for i in range(len(clusters)):
            print("cluster " + str(i) + "has x: = " + str(clusters[i].top_center_pose.position.x) + " and y = " +  str(clusters[i].top_center_pose.position.y))

            dist = clusters[i].top_center_pose.position.x #(clusters[i].top_center_pose.position.x**2.0 + clusters[i].top_center_pose.position.y**2.0) ** 0.5

            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        print("picked cluster " + str(closest_idx))

        return closest_idx







if __name__ == '__main__':
    rospy.init_node('cleanup_node')

    # ROSTOPIC ECHO /AMCL_POSE TO GET POSES
    manipulation_areas = [
        ManipulationArea("back_room", (-3, 0), (0.0, 0.0, -0.997891458136, 0.0649048363319)),
        ManipulationArea("table1", (2, 0), (0.0, 0.0, 0.0, 1.0)),
    ]
    pickup_areas = [1]
    cleanup = Cleanup(manipulation_areas, pickup_areas, True)
    cleanup.run_cleanup()
    rospy.spin()
