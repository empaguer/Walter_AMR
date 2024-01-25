#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
import time

rospy.loginfo("GOD")
rospy.init_node("odom_publisher")
rospy.loginfo("hello2")
odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=5)

listener = tf.TransformListener()

while not rospy.is_shutdown():

    # read the transform
    try:
        (trans, rot) = listener.lookupTransform("map", "base_footprint", rospy.Time(0))

        odom_msg = Odometry()

        odom_msg.pose.pose.position.x = trans[0]
        odom_msg.pose.pose.position.y = trans[1]
        odom_msg.pose.pose.position.z = trans[2]

        odom_msg.pose.pose.orientation.x = rot[0]
        odom_msg.pose.pose.orientation.y = rot[1]
        odom_msg.pose.pose.orientation.z = rot[2]
        odom_msg.pose.pose.orientation.w = rot[3]
        print("hello i am trying")

        odom_publisher.publish(odom_msg)

    except:
        rospy.loginfo("hello")
        pass

    rospy.sleep(0.5)
