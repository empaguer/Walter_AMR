#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

rospy.init_node("round_movement_node")

vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

while not rospy.is_shutdown():
    print("sending vel")
    vel_msg = Twist()
    vel_msg.linear.x = 0.15
    vel_msg.angular.z = 0.4
    vel_pub.publish(vel_msg)
    time.sleep(5)

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)
    time.sleep(3)



