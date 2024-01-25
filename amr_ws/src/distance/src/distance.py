#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image as msg_Image
from darknet_ros_msgs.msg import BoundingBoxes as msg_BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
import numpy as np


class ImageListener:
    """gets image from depth camera and validates distance to people"""

    def __init__(self, topic1, topic2):
        self.topic1 = topic1
        self.topic2 = topic2
        self.bridge = CvBridge()
        self.sub1 = rospy.Subscriber(topic1, msg_Image, self.image_depth_callback)
        self.sub2 = rospy.Subscriber(
            topic2, msg_BoundingBoxes, self.bounding_boxes_callback
        )
        self.pub = rospy.Publisher("/presence", Bool, queue_size=10)

    def image_depth_callback(self, depth):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            self.depth_array = np.array(depth_image, dtype=np.float32)
            print(self.depth_array)
            # hello_str = dist_to_center
        except CvBridgeError as _e:
            print(_e)

    def bounding_boxes_callback(self, data):
        """gets bounding boxes values from darknet_ros"""
        distancia_min = 10000
        print(data)
        for box in data.bounding_boxes:
            coords = [box.ymin, box.ymax, box.xmin, box.xmax]
            centerx, centery = (np.average(coords[:2]), np.average(coords[2:]))
            distancia = self.depth_array[int(centerx), int(centery)]
            print(distancia)
            if distancia < distancia_min:
                distancia_min = distancia
            if distancia_min <= 4000 and distancia_min > 0.0:
                boolean = True
                hello_str = boolean
                rospy.loginfo(hello_str)
                self.pub.publish(hello_str)
            else:
                boolean = False
            # hello_str = dist_to_center


if __name__ == "__main__":
    rospy.init_node("depth_image_processor")
    topic1 = "/d435/depth/image_raw"
    topic2 = "/darknet_ros/bounding_boxes"
    listener = ImageListener(topic1, topic2)
    rospy.spin()
