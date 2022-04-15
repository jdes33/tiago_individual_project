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
    x,y,z = (0, 0, 1)

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
