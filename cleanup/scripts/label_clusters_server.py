#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool


##
#
#      FOR NOW THIS IS JUST A TEST NOT A SERVICE NOR CODE THAT MAY BE USED IN SERVICE (MIGHT NOT NEED TO GET FULL CLUSTERS HERE)
#
#
#
#
#
#

class NumberCounter:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
    def callback_number(self, msg):
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        self.pub.publish(new_msg)
    def callback_reset_counter(self, req):
        if req.data:
            self.counter = 0
            return True, "Counter has been successfully reset"
        return False, "Counter has not been reset"
if __name__ == '__main__':
    rospy.init_node('number_counter')

    # call cluster service to get clusters
    rospy.wait_for_service('/get_clusters')
    try:
        get_clusters_service = rospy.ServiceProxy('/get_clusters', GetClusters)
        ##p = Point(resp1.target_pose.position)
        clusters = get_clusters_service()

        if clusters:
            rospy.loginfo("obtained clusters")
    except rospy.ServiceException as e:
        print("GetClusters service call failed: %s"%e)

    # call yolo service to get boxes
        #ros::ServiceClient client = nh_.serviceClient<yolo4::DetectObjects>("/yolo_server/detect_objects");

    rospy.wait_for_service('/yolo_server/detect_objects')
    try:
        get_detections_service = rospy.ServiceProxy('/yolo_server/detect_objects', DetectObjects)
        ##p = Point(resp1.target_pose.position)
        detections = get_detections_service()

        if detections:
            rospy.loginfo("obtained yolo detections")
    except rospy.ServiceException as e:
        print("DetectObjects service call failed: %s"%e)


    ## NEED TO GO FROM CLUSTERS/POINT CLOUDS TO IMAGE POINTS
    

    NumberCounter()
    rospy.spin()



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
