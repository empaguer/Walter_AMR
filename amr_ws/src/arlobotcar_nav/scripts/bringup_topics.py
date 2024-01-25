#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, String

def talker():
    # Initialize the publisher node
    rospy.init_node('topic_initializers', anonymous=True)
    #hello_str = Int16()
    # Create a publisher object
    mapping_pub = rospy.Publisher('mapping_node', String, queue_size=10)
    navigation_pub = rospy.Publisher('navigation_node',String, queue_size=10)
    calibration_pub = rospy.Publisher('calibration_node',String,queue_size=10)
    hello_str = "hello"
    
    # Set a rate for the publishing
    rate = rospy.Rate(10)  # 10hz
    mapping_pub.publish(hello_str)
    navigation_pub.publish(hello_str)
    calibration_pub.publish(hello_str)
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass