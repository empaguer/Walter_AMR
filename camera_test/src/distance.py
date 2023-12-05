#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image as msg_Image
from darknet_ros_msgs.msg import BoundingBoxes as msg_BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
import sys
from std_msgs.msg import Bool
import pyrealsense2 as rs
import numpy as np
import os


class ImageListener:
    def __init__(self, topic1, topic2):
        self.topic1 = topic1
        self.topic2 = topic2
        self.bridge = CvBridge()
        self.sub1 = rospy.Subscriber(topic1, msg_Image, self.imageDepthCallback)
        self.sub2 = rospy.Subscriber(
            topic2, msg_BoundingBoxes, self.boundingBoxesCallback)
        self.pub = rospy.Publisher('/presence', Bool, queue_size=10)
    def imageDepthCallback(self, depth):
        boolean = False
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            self.depth_array = np.array(depth_image, dtype=np.float32)
            # hello_str = dist_to_center
        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(e)
        return

    def boundingBoxesCallback(self,data):
        distancia_min = 10000
        for box in data.bounding_boxes:
            coords = [box.ymin, box.ymax, box.xmin, box.xmax]
            centerx,centery = (np.average(coords[:2]),np.average(coords[2:]))
            distancia = self.depth_array[int(centerx), int(centery)]
            print(distancia)
            if distancia < distancia_min:
                distancia_min = distancia
            if distancia_min <= 3000 and distancia_min >0.0:
                boolean = True
                hello_str = boolean
                rospy.loginfo(hello_str)
                self.pub.publish(hello_str)
            else:
                boolean = False
            # hello_str = dist_to_center

if __name__ == '__main__':
    rospy.init_node("depth_image_processor")
    topic1 = '/camera/depth/image_rect_raw'
    topic2 = '/darknet_ros/bounding_boxes'
    listener = ImageListener(topic1, topic2)
    rospy.spin()
