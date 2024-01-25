#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
import time


rospy.init_node("odom_publisher")

odom_publisher = rospy.Publisher("/odom2", Odometry, queue_size=5)

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

        odom_publisher.publish(odom_msg)

        rospy.loginfo(trans)

    except:
        rospy.loginfo("Waiting cartographer to publish map transformate")
        pass

    rospy.sleep(0.5)
