#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
def callback(msg):
    rospy.loginfo(msg.data)
def listener():
    rospy.init_node("esp_listener")
    rospy.Subscriber("esp_velocity",Float32,callback)
    rospy.spin()
if __name__ == "__main__":
    listener()