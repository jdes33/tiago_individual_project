#!/usr/bin/env python
import rospy
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo

def image_callback(image):
    pass

def info_callback(info):
    pass


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)

	rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)
	#rospy.Subscriber(topic_blob, Blobs, self.blob_callback)
	rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, info_callback)

    ci = sensor_msgs.msg.CameraInfo()
    cam = PinholeCameraModel()
    cam.fromCameraInfo(ci)
    print cam.project3dToPixel((0,0,0))
    print cam.tfFrame()
