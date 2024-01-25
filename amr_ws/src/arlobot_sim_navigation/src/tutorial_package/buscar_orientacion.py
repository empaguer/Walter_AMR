#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from math import sqrt
my_id = 9999
# Define a callback function to get the robot's current position
is_position_reached = False
timer_active = True
def timer_callback(event):
    global is_position_reached, timer_active
    if not is_position_reached:
        rospy.loginfo("30 seconds have passed, aborting the task.")
        # Code to abort the task
        timer_active = False


def move_to_orientation(id):
    timer = rospy.Timer(rospy.Duration(30), timer_callback, oneshot=True)
    global is_position_reached, timer_active
    #global x, y
    #rospy.init_node('move_to_orientation', anonymous=True)
    # Define the AprilTag detection topic you want to subscribe to
    detection_topic = "/tag_detections"
    rospy.Subscriber(detection_topic, AprilTagDetectionArray, april_tag_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # Control loop rate (10 Hz)
    while not rospy.is_shutdown() and timer_active:
        #Poner aqui timer para mandar false
        cmd_vel = Twist()
        if my_id!=id:
            cmd_vel.angular.z= 0.25
            cmd_vel_pub.publish(cmd_vel)
            # aqui timer
        else:
            cmd_vel.angular.z= 0
            cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo(" ID FOUND, STOPPING ROTATION")
            is_position_reached = True
            break
        rate.sleep()
    #TODOO WITH CTRL C THE WHEELS SHOULD STOP ROTATING
    final = Twist()
    final.angular.z=0
    cmd_vel_pub.publish(final)
    return is_position_reached
def april_tag_callback(data):
    global my_id
    if data.detections:
        detection = data.detections[0]
        try:
            my_id = detection.id[0]
        except Exception:
            pass
#if __name__ == '__main__':
#    try:
#        move_to_orientation(0)

#    except rospy.ROSInterruptException:
#        pass