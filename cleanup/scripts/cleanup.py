#!/usr/bin/env python
# Author: Jason De Souza <k1922008@kcl.ac.uk>
from __future__ import print_function
import rospy



from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
class ManipulationArea:

    def __init__(self, name, position, orientation):

        self.name = name

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

        print("moving to " + self.name)
        # use move base to go
        self.move_base_client.send_goal(self.goal)
        self.move_base_client.wait_for_result()
        print("i think it's reached the destination, proceeding with inspect surroungins")
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





import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2
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
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
        except tf2.ExtrapolationException as ex:
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


            # get the cluster points, pick one and add text. CHANGE TO PICK SURFACE POSE LATER MAYBE
            cloud_points_base_frame = list(point_cloud2.read_points(cluster.pointcloud, skip_nans=True, field_names = ("x", "y", "z")))
            text_marker_data.append([max_label, cloud_points_base_frame[-1][0], cloud_points_base_frame[-1][1], cloud_points_base_frame[-1][2] + 0.1])


        ## IF YOU PREFER MAYBE MAKE MARKER CLASS LIKE THE CONSTRUCT, BUT I THINK THIS IS OKAY
        self.publish_text_markers(text_marker_data, 5)  # display text markers above each cluster for 5 seconds
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








from pcd_proc.srv import GetClusters
class Cleanup:

    def __init__(self, manipulation_areas, pickup_indexes, label_all = False):
        self.manipulation_areas = manipulation_areas
        self.pickup_indexes = pickup_indexes
        self.object_destinations = {"unknown":0, "apple": 0, "banana":0, "fork":0}
        # for debugging/visual purpose can label all clusters instead of just the one to be picked
        self.label_all = label_all
        self.cluster_labeler = ClusterLabeler()



    def run_cleanup(self):

        for pickup_index in self.pickup_indexes:
            clean = False
            while not clean:
                self.manipulation_areas[pickup_index].go(True)
                clusters = self.get_clusters()

                if len(clusters) == 1:
                    clean = True
                    rospy.loginfo("Cleaning last item at, shouldn't need to visit here " + self.manipulation_areas[pickup_index].name + " again.")
                elif len(clusters) == 0:
                    # should only get here if something else moved an item while the robot was away
                    clean = True
                    rospy.loginfo("Seems there aren't any more items at " + self.manipulation_areas[pickup_index].name + ".")
                    break

                closest_cluster_index = self.get_closest(clusters)
                label = ""
                if self.label_all:
                    labels = self.cluster_labeler.label_clusters(clusters)
                    label = labels[closest_cluster_index]
                else:
                    label = self.cluster_labeler.label_clusters([clusters[closest_cluster_index]])[0]

                print("picking object called " + label)
                self.pickup(clusters[closest_cluster_index])
                destination = self.object_destinations[label]
                self.manipulation_areas[destination].go(False)
                print("placing object called " + label)
                self.place()


    def get_clusters(self):
        # call cluster service to get clusters
        rospy.wait_for_service('/get_clusters')
        try:
            get_clusters_service = rospy.ServiceProxy('/get_clusters', GetClusters)
            ##p = Point(resp1.target_pose.position)
            get_clusters_response = get_clusters_service()
            print("Got ", len(get_clusters_response.clusters), " clusters")

            if get_clusters_response:
                rospy.loginfo("obtained clusters")
        except rospy.ServiceException as e:
            print("GetClusters service call failed: %s"%e)

        return get_clusters_response.clusters

    def label_clusters(self, clusters):
        #probs should make a service and call it here
        # the service should take a list of clusters (JUST NEEDS POINTCLOUD & MAYBE FRAME INFO)
        # ther service should return list of labels
        labels = ["bannana", "apple", "fork"]
        labels = ["apple"]
        return labels

    def get_closest(self, clusters):
        return 0 # NEED TO IMPLEMENT THIS PROPERLY

    def pickup(self, cluster):
        # NEED TO DO PLANNING SCENE STUFF
        # NEED TO DO HAF GRASPING STUFF
        # PROBS TOO MUCH TO DO IN ONE FUNCTION UNLESS YOU MAKE PLANNING SCENE AN ATTRIBUTE OF THIS CLASS
        # THINK ABOUT WHATIS BEST WAY
        # INPUTS WOULD BE CLOUD, THERE WOULD BE NO OUTPUT (OR MAYBE BOOL FOR SUCCESS, BUT YOU CAN'T EVEN CHECK SUCCESS SO MAYBE NOT FOR NOW)
        # MAYBE SHOULD DO A CLASS WHICH CONTROLS BOTH PICK AND PLACE, AS YOU MAY NEED TO CARRY OVER INFORMATION????
        # MAYBE CLASS/SERVER CALLED PICK_PLACE:
        #    ATTRIBUTES ARE PLANNING SCENE, ....
        #    METHODS ARE PICK() AND PLACE(), INIT(CLUSTER), ....
        # USE CTRL+/ IF YOU WANT TO UNCOMMENT THE BELOW (THO DON'T EXPECT IT TO WORK IT'S SIMPLY COPIED FROM PICK_TEST.PY)
            # scene = PlanningSceneInterface()
            # robot = RobotCommander()
            # right_arm = MoveGroupCommander("arm_torso")
            # print "============ Robot Groups:"
            # print robot.get_group_names()
            # print right_arm.get_named_targets()
            # right_gripper = MoveGroupCommander("gripper")
            # print right_gripper.get_named_targets()
            # rospy.sleep(1)
            #
            #
            # rospy.loginfo("Creating publishers to torso and head controller...")
            # torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1, latch=True)
            # head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1, latch=True)
            # rospy.loginfo("Creating publisher for haf_grasping")
            # haf_pub = rospy.Publisher('/haf_grasping/input_pcd_rcs_path', String, queue_size=1, latch=True)
            # haf_sub = rospy.Subscriber('/haf_grasping/grasp_hypothesis_with_eval', String, process_haf_grasp)
            #
            # rospy.sleep(1)  # LATCHING DONT SEEM TO FIX SO I THINK JUST SLEEP FOR A BIT, MAYBE JUST MAKE LOOK DOWN MOTION OR ACTION INSTEAD
            # rospy.loginfo("Moving head down")
            # jt = JointTrajectory()
            # jt.joint_names = ['head_1_joint', 'head_2_joint']
            # jtp = JointTrajectoryPoint()
            # jtp.positions = [0.0, -0.98]
            # jtp.time_from_start = rospy.Duration(2)
            # jt.points.append(jtp)
            # head_cmd.publish(jt)
            #
            # rospy.loginfo("Moving torso up/down")
            # jt = JointTrajectory()
            # jt.joint_names = ['torso_lift_joint']
            # jtp = JointTrajectoryPoint()
            # jtp.positions = [0.0] #[0.34]
            # jtp.time_from_start = rospy.Duration(2)
            # jt.points.append(jtp)
            # torso_cmd.publish(jt)
            #
            # rospy.sleep(3.0)  # SLEEP TO WAIT TIL LOOK DOWN ACTION DONE
            # rospy.loginfo("Done")
            #
            #
            # # clean the scene
            # scene.remove_world_object("table")
            # scene.remove_world_object("MApart")
            # scene.remove_attached_object()  # removes all previously attached objects
            #
            #
            # # FILTER, SEGMENT AND CLUSTER TO FIND 3D OBJECT BOUNDING BOX
            # rospy.wait_for_service('find_pick')
            # try:
            #     pick_as = rospy.ServiceProxy('find_pick', GetTargetPose)
            #     resp1 = pick_as()
            #     rospy.loginfo("Got target pose: " + str(resp1.target_pose))
            # except rospy.ServiceException as e:
            #     print("Service call failed: %s"%e)
            #
            #
            # # COMPUTE GRASP USING HAF GRASPING
            # rospy.wait_for_service('/haf_grasping/set_grasp_center')
            # try:
            #     set_center_service = rospy.ServiceProxy('/haf_grasping/set_grasp_center', GraspSearchCenter)
            #     ##p = Point(resp1.target_pose.position)
            #     resp2 = set_center_service(resp1.target_pose.position)
            #     if resp2:
            #         rospy.loginfo("Set search center")
            # except rospy.ServiceException as e:
            #     print("Service call failed: %s"%e)
            #
            # msg = String()
            # msg.data = rospack.get_path('cleanup') + "/data/object/detected_object.pcd"
            # haf_pub.publish(msg)
            #
            #
            # length = abs(resp1.max_pt.x - resp1.min_pt.x)
            # width = abs(resp1.max_pt.y - resp1.min_pt.y)
            # height = abs(resp1.max_pt.z - resp1.min_pt.z)
            # print length, width, height
            # print resp1.min_pt.x, resp1.min_pt.y, resp1.min_pt.z
            # print resp1.max_pt.x, resp1.max_pt.y, resp1.max_pt.z
            #
            # rospy.sleep(1)
            #
            # # publish a demo scene
            # p = PoseStamped()
            # p.header.frame_id = robot.get_planning_frame()
            #
            # # add a table
            # p.pose.position.x = 0.6
            # p.pose.position.y = -0.2
            # p.pose.position.z = 0.24
            # #scene.add_box("table", p, (0.5, 1.5, 0.48))
            #
            # # add an object to be grasped
            # p.pose.position.x = resp1.min_pt.x + length / 2.0
            # p.pose.position.y = resp1.min_pt.y + width / 2.0
            # p.pose.position.z =  resp1.target_pose.position.z / 2.0# resp1.min_pt.z + height / 2.0#
            # scene.add_box("MApart", p, (length, width, resp1.target_pose.position.z))#resp1.target_pose.position.z))
            #
            #
            # rospy.sleep(1)
            # rospy.loginfo("added objects")
            #
            # # define a list of grasps, for now a single grasp
            # grasps = [Grasp()]
            #
            # grasps[0].id = "test"
            #
            # # set the pose of the Pose of the end effector in which it should attempt grasping
            # grasps[0].grasp_pose = PoseStamped()
            # grasps[0].grasp_pose.header.frame_id = "base_footprint"
            # #q = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
            # #print "GOING TO", p.pose.position.x, p.pose.position.y, p.pose.position.z# + 0.23
            # #grasps[0].grasp_pose.pose = Pose(Point(p.pose.position.x, p.pose.position.y, 0.24), Quaternion(*q))
            # while haf_ready == False:
            #     print "waiting for haf grasp"
            #     rospy.sleep(5)
            #
            # if current_haf_grasp["val"] == -20:
            #     print "NO GRASP FOUND BY HAF, using basic grasp"
            #     q = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
            #     print "GOING TO", p.pose.position.x, p.pose.position.y, p.pose.position.z# + 0.23
            #     grasps[0].grasp_pose.pose = Pose(Point(p.pose.position.x, p.pose.position.y, 0.24), Quaternion(*q))
            # else:
            #     print "Using haf grasp"
            #     grasps[0].grasp_pose.pose.position.x, grasps[0].grasp_pose.pose.position.y, grasps[0].grasp_pose.pose.position.z = current_haf_grasp["gcp"]
            #     grasps[0].grasp_pose.pose.position.z = 0.23#0.23  ## this will make almost touch floor # this will make tip of gripper align with position
            #     q = tf.transformations.quaternion_from_euler(0, 1.57, -math.radians(current_haf_grasp["roll"]))
            #     grasps[0].grasp_pose.pose.orientation = Quaternion(*q)
            #     print "hey"
            #     print grasps[0].grasp_pose.pose
            # haf_ready = False
            # rospy.sleep(2)
            #
            # # define the pre-grasp approach, this is used to define the direction from which to approach the object and the distance to travel
            # grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint"
            # grasps[0].pre_grasp_approach.direction.vector.x = 0.0
            # grasps[0].pre_grasp_approach.direction.vector.y = 0.0
            # grasps[0].pre_grasp_approach.direction.vector.z = -1.0
            # grasps[0].pre_grasp_approach.min_distance = 0.1
            # grasps[0].pre_grasp_approach.desired_distance = 0.5
            #
            # # define the pre-grasp posture, this defines the trajectory position of the joints in the end effector group before we go in for the grasp
            # # WE WANT TO OPEN THE GRIPPER, YOU CAN TAKE SOME OF THIS AND MAKE INTO CLOSE GRIPPER FUNCTION
            # pre_grasp_posture = JointTrajectory()
            # pre_grasp_posture.header.frame_id = "arm_tool_link"
            # pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
            # jtpoint = JointTrajectoryPoint()
            # #dist = sum([(current_haf_grasp["gp1"][i] - current_haf_grasp["gp2"][i])**2 for i in range(3)]) ** 0.5
            # #jtpoint.positions = [dist/2.0, dist/2.0]
            # jtpoint.positions = [0.04, 0.04]
            # jtpoint.time_from_start = rospy.Duration(2.0)
            # pre_grasp_posture.points.append(jtpoint)
            # grasps[0].pre_grasp_posture = pre_grasp_posture
            #
            # # set the grasp posture, this defines the trajectory position of the joints in the end effector group for grasping the object
            # # WE WANT TO CLOSE THE GRIPPER
            # grasp_posture = copy.deepcopy(pre_grasp_posture)
            # grasp_posture.points[0].time_from_start = rospy.Duration(2.0 + 1.0)
            # jtpoint2 = JointTrajectoryPoint()
            # dist = sum([(current_haf_grasp["gp1"][i] - current_haf_grasp["gp2"][i])**2 for i in range(3)]) ** 0.5
            #
            # finger_dist = (dist * 0.8)/2.0 # 0.004#0.003
            # finger_dist = 0.0
            # print "finger dist = ", str(finger_dist)
            #
            # jtpoint2.positions = [finger_dist, finger_dist]
            # jtpoint2.time_from_start = rospy.Duration(2.0 + 1.0 + 3.0)
            # #jtpoint2.effort.append(1.0)
            # #jtpoint2.effort.append(1.0)
            # grasp_posture.points.append(jtpoint2)
            # grasps[0].grasp_posture = grasp_posture
            #
            # # set the post-grasp retreat, this is used to define the direction in which to move once the object is grasped and the distance to travel.
            # grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint"
            # grasps[0].post_grasp_retreat.direction.vector.x = 0.0
            # grasps[0].post_grasp_retreat.direction.vector.y = 0.0
            # grasps[0].post_grasp_retreat.direction.vector.z = 1.0
            # grasps[0].post_grasp_retreat.desired_distance = 0.25
            # grasps[0].post_grasp_retreat.min_distance = 0.01
            #
            # # links_to_allow_contact: ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]
            # #grasps[0].allowed_touch_objects = ["table", "MApart"]  # THIS LEFT EMPTY ON THINGY
            # #grasps[0].links_to_allow_contact = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]
            # #Don't restrict contact force
            # grasps[0].max_contact_force = 0.0
            # print "max contact force", grasps[0].max_contact_force
            #
            # # pick the object
            # right_arm.pick("MApart", grasps)
            #
            #
            # rospy.loginfo("Moving head back up")
            # jt = JointTrajectory()
            # jt.joint_names = ['head_1_joint', 'head_2_joint']
            # jtp = JointTrajectoryPoint()
            # jtp.positions = [0.0, 0.0]
            # jtp.time_from_start = rospy.Duration(2.0)
            # jt.points.append(jtp)
            # head_cmd.publish(jt)
            # rospy.sleep(2.5)  # SLEEP TO WAIT TIL LOOK DOWN ACTION DONE
            # rospy.loginfo("Done")
            #
            # rospy.loginfo("MOVING TO HOME POSITION, HOPEFULLY WITH OBJECT")
            # goal = PlayMotionGoal()
            # goal.motion_name = 'home'
            # goal.skip_planning = True
            # client.send_goal(goal)
            # client.wait_for_result(rospy.Duration(10.0))


        pass

    def place(self):
        ## NEED SERVICE TO GET POINTCLOUD PLANE??? MAYBE JUST MANUALLY STORE FOR EACH OBJECT
        ## IF YOU HAVE TO MAKE SERVICE FOR PLANE THEN MAYBE JUST MAKE IT IN THIS PACKAGE IN SRC
        pass




if __name__ == '__main__':
    rospy.init_node('cleanup_node')

    # ROSTOPIC ECHO /AMCL_POSE TO GET POSES
    manipulation_areas = [
        ManipulationArea("back_room", (-2.86942732871, -0.219775309016), (0.0, 0.0, -0.997891458136, 0.0649048363319)),
        ManipulationArea("table1", (2, 0), (0.0, 0.0, 0.0, 1.0)),
    ]
    pickup_areas = [1]
    cleanup = Cleanup(manipulation_areas, pickup_areas, True)
    cleanup.run_cleanup()
    rospy.spin()
