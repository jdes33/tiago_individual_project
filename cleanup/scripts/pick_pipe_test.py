import rospy
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation

rospy.init_node("moveit_py")
# provide arm group and gripper group names
# also takes a third parameter "plan_only" which defaults to False
p = PickPlaceInterface("arm", "gripper")

g = Grasp()
# fill in g
g.id = "name for grasp"
g.pre_grasp_posture = 


# setup object named object_name using PlanningSceneInterface
p.pickup("object_name", [g, ], support_name = "supporting_surface")

l = PlaceLocation()
# fill in l
p.place("object_name" [l, ], goal_is_eef = True, support_name = "supporting_surface")
