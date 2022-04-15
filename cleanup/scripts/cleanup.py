#!/usr/bin/env python3
# Author: Jason De Souza <k1922008@kcl.ac.uk>

import rospy



from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
class ManipulationArea:

    def __init__(self, name, position, orientation):

        self.name = name

        self.client = SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
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

        # use move base to go
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        #self.inspect_surroundings()  # to build octomap

        rospy.loginfo("moved to " + self.name)
        pass

    #def inspect_surroundings(self):
#        client = SimpleActionClient("play_motion", PlayMotionAction)
#        client.wait_for_server()
#        rospy.loginfo("...connected to play_motion client")
#        goal = PlayMotionGoal()#
#        goal.motion_name = 'home'#'inspect_surroundings'
    #    goal.skip_planning = True
#        client.send_goal(goal)
    #    client.wait_for_result(rospy.Duration(10.0))





class Cleanup:

    def __init__(self, manipulation_areas, pickup_indexes):
        self.manipulation_areas = manipulation_areas
        self.pickup_indexes = pickup_indexes
        self.object_destinations = {"unknown":0, "apple": 1}
        self.label_all = False



    def run_cleanup(self):

        for pickup_index in self.pickup_indexes:
            clean = False
            while not clean:
                self.manipulation_areas[pickup_index].go(True)
                clusters = self.get_clusters()
                closest_cluster_index = self.get_closest(clusters)
                label = ""
                if self.label_all:
                    labels = self.label_clusters(clusters)
                    label = labels[closest_cluster_index]
                else:
                    label = self.label_clusters([clusters[closest_cluster_index]])[0]

                self.pickup(cluster)
                destination = self.object_destinations[label]
                self.manipulation_areas[destination].go()
                self.place()

    def get_clusters(self):
        pass

    def label_clusters(self):
        pass

    def get_closest(self, clusters):
        pass

    def pickup(self):
        pass

    def place(self):
        pass




if __name__ == '__main__':
    rospy.init_node('cleanup_node')

    manipulation_areas = [
        ManipulationArea("table1", (2,0), (0.0, 0.0, 0.0, 1.0)),
    ]
    pickup_areas = [0]
    cleanup = Cleanup(manipulation_areas, pickup_areas)
    cleanup.run_cleanup()
    rospy.spin()
