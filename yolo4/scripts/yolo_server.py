#!/usr/bin/env python3
import cv2
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge
from cv_bridge3 import cv2
from yolo4.srv import DetectObjects,DetectObjectsResponse
from yolo4.msg import Detection

class Yolo4:

    def __init__(self):
        self.service = rospy.Service(rospy.get_name() + '/detect_objects', DetectObjects, self.handle_request)

    def handle_request(self, req):
        res = DetectObjectsResponse()

        # get paths to data needed for YOLO
        rospack = rospkg.RosPack()
        package_name = "yolo4"
        yolo_config_path = rospack.get_path(package_name) + "/config/yolov4.cfg"
        weights_path = rospack.get_path(package_name) + "/config/yolov4.weights"
        classes_path = rospack.get_path(package_name) + "/config/coco.names"

        msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)

        # transform sensor_msgs/Image to cv image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        with open(classes_path, 'r') as f:
            classes = f.read().splitlines()

        # load network
        net = cv2.dnn.readNetFromDarknet(yolo_config_path, weights_path)
        model = cv2.dnn_DetectionModel(net)
        model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)

        # preform detection
        classIds, scores, boxes = model.detect(img, confThreshold=0.4, nmsThreshold=0.4)
        for (classId, score, box) in zip(classIds, scores, boxes):
            cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                         color=(0, 255, 0), thickness=2)

            text = '%s: %.2f' % (classes[classId], score)
            cv2.putText(img, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                       color=(0, 255, 0), thickness=2)

            print(classes[classId], box[0], box[1], box[2], box[3], score)
            detection = Detection(classes[classId], box[0], box[1], box[2], box[3], score)

            res.detections.append(detection)



        cv2.imwrite("object-detection.jpg", img)
        print("saved")

        cv2.imshow('Image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return res




if __name__ == '__main__':

    rospy.init_node('yolo_server')
    try:
        objectDetector = Yolo4()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
