#!/usr/bin/env python
import rospy

from haf_grasping.srv import GraspSearchCenter
from yolo4.srv import DetectObjects
from pcd_proc.srv import GetClusters

#from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# for 3d point to pixel
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo


import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2

from visualization_msgs.msg import MarkerArray, Marker


def generate_text_markers(labels):
    marker_idx = 0
    marker_array_msg = MarkerArray()
    for i in range(len(labels)):
        marker = Marker()
        marker.header.frame_id = "base_footprint" # MIGHT WANNA CHANGE TO MAP
        marker.id = i
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text = labels[i][0]
        ##marker.pose = Pose()
        marker.pose.position.x = labels[i][1]
        marker.pose.position.y = labels[i][2]
        marker.pose.position.z = labels[i][3]
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
        marker.lifetime = rospy.Duration(5)
    # Set the namespace and id for this marker.  This serves to create a unique ID
    # Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "cluster_label"
        marker_array_msg.markers.append(marker)
    return marker_array_msg

##
#
#      FOR NOW THIS IS JUST A TEST NOT A SERVICE NOR CODE THAT MAY BE USED IN SERVICE (MIGHT NOT NEED TO GET FULL CLUSTERS HERE)
#
#
#
#
#
#


if __name__ == '__main__':




    rospy.init_node('label_clusters_server')
    tf_buffer = tf2_ros.Buffer()
    # NOTE: this must be created some time before you use it or you could get frame not exist warning
    # source for NOTE: https://answers.ros.org/question/192570/tftransformlistenertransformpose-exception-target_frame-does-not-exist/
    listener = tf2_ros.TransformListener(tf_buffer)
    print("hey 1")
    # call cluster service to get clusters
    rospy.wait_for_service('/get_clusters')
    try:
        get_clusters_service = rospy.ServiceProxy('/get_clusters', GetClusters)
        ##p = Point(resp1.target_pose.position)
        get_clusters_response = get_clusters_service()
        print "Got ", len(get_clusters_response.clusters)

        if get_clusters_response:
            rospy.loginfo("obtained clusters")
    except rospy.ServiceException as e:
        print("GetClusters service call failed: %s"%e)



    print("hey 2")

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
            print "Got ", len(get_detections_response.detections)
            print("hey 2.3")

    except rospy.ServiceException as e:
        print("DetectObjects service call failed: %s"%e)




    print("hey 3")

    ## NEED TO GO FROM CLUSTERS/POINT CLOUDS TO IMAGE POINTS
    camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
    print camera_info.width
    print camera_info.height

    cam = PinholeCameraModel()
    cam.fromCameraInfo(camera_info)
    # convert pointclouds (which are in base_footprint) to xtion_rgb_optical_frame
    try:
        trans = tf_buffer.lookup_transform("xtion_rgb_optical_frame","base_footprint", rospy.Time())
    except tf2.LookupException as ex:
        rospy.logwarn(ex)
    except tf2.ExtrapolationException as ex:
        rospy.logwarn(ex)


    transformed_pointcloud = do_transform_cloud(get_clusters_response.clusters[-1].pointcloud, trans)

    pixels = set()


    print("hey 4")

    cloud_points = list(point_cloud2.read_points(transformed_pointcloud, skip_nans=True, field_names = ("x", "y", "z")))

    for point in cloud_points:
        pixels.add(cam.project3dToPixel((point[0], point[1], point[2])))

    print "test point", cloud_points[-1]
    print "number of pixels in mask/region = ", len(pixels)
    average_x = sum([p[0] for p in pixels]) / len(pixels)
    average_y = sum([p[1] for p in pixels]) / len(pixels)
    print "average pixel = ", average_x, "  ", average_y

    #print "converting 3d point " ,x,y,z , " to pixel:"
    #print cam.project3dToPixel((x,y,z))

    #labels = [["HEY", cloud_points_base_frame[-1][0], cloud_points_base_frame[-1][1], cloud_points_base_frame[-1][2]]]

    print("detections: ")
    for detection in get_detections_response.detections:
        print detection.label

    labels = []
    for cluster in get_clusters_response.clusters:

        transformed_pointcloud = do_transform_cloud(cluster.pointcloud, trans)
        pixels = set()
        cloud_points = list(point_cloud2.read_points(transformed_pointcloud, skip_nans=True, field_names = ("x", "y", "z")))
        for point in cloud_points:
            pixels.add(cam.project3dToPixel((point[0], point[1], point[2])))


        # go through each yolo and find the one with max overlay
        max_percentage_filled = 0
        max_label = "unknown"
        # MAYBE ADD A THRESHOLD TOO? i.e percentage of points in detection box
        for detection in get_detections_response.detections:
            current_overlap = 0
            for pixel in pixels:
                if pixel[0] >= detection.x and pixel[0] <= detection.x + detection.width and pixel[1] >= detection.y and pixel[1] < detection.y + detection.height:
                    current_overlap += 1

            percentage_filled = (1.0 * current_overlap) / (detection.width * detection.height)
            if percentage_filled > max_percentage_filled:
                max_percentage_filled = percentage_filled
                max_label = detection.label
        print max_label, " picked as fills ", max_percentage_filled


        # get the cluster points, pick one and add text. CHANGE TO PICK SURFACE POSE LATER MAYBE
        cloud_points_base_frame = list(point_cloud2.read_points(cluster.pointcloud, skip_nans=True, field_names = ("x", "y", "z")))
        labels.append([max_label, cloud_points_base_frame[-1][0], cloud_points_base_frame[-1][1], cloud_points_base_frame[-1][2] + 0.1])




    marker_pub = rospy.Publisher('JASONvisualization_marker', MarkerArray, queue_size=5)

#    rate = rospy.Rate(1)
    #while not rospy.is_shutdown():
#        marker_pub.publish(generate_text_markers(labels))
#        rate.sleep()

    # publish just once: source: https://answers.ros.org/question/267199/how-can-i-publish-exactly-once-when-the-node-is-run/
    try:
        marker_pub = rospy.Publisher('JASONvisualization_marker', MarkerArray, queue_size=5)
        import time
        #time.sleep(1)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            connections = marker_pub.get_num_connections()
            rospy.loginfo('Connections: %d', connections)
            if connections > 0:
                marker_pub.publish(generate_text_markers(labels))
                rospy.loginfo('Published')
                break
            rate.sleep()
    except rospy.ROSInterruptException, e:
        raise e

    print cam.tfFrame()

    print "DONE"
    #rospy.spin()



# maybe service that identifies a single cluster
# that way you can call on all cluster IFF you want, or just a portion
# maybe main script will use size of picksurfact to determine if it should call identify service


# only need pointcloud for this, not pick surfacr or anyting
# either pass in frames or pass in list of pointclods so you you don't have to call yolo multiple times
# will it show a marker? or a rather a list of markers with label?? it doesnt have to but it may be better
# will it also be in charge of pose est? -> no i dont think so

# Service:
# request: a list of pointclouds ()
# response: a list of labelled pointclouds

# Get clusters
# Get yolo frames
#
