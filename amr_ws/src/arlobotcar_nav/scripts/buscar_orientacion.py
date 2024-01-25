#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from math import sqrt
my_id = 9999
# Define a callback function to get the robot's current position

def move_to_desired_position(id):
    global x, y
    rospy.init_node('move_to_desired_position', anonymous=True)
    # Define the AprilTag detection topic you want to subscribe to
    detection_topic = "/tag_detections"
    rospy.Subscriber(detection_topic, AprilTagDetectionArray, april_tag_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # Control loop rate (10 Hz)
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        if my_id!=id:
            cmd_vel.angular.z= 0.25
            cmd_vel_pub.publish(cmd_vel)
        else:
            cmd_vel.angular.z= 0
            cmd_vel_pub.publish(cmd_vel)
            break
        rate.sleep()
    #TODOO WITH CTRL C THE WHEELS SHOULD STOP ROTATING
    final = Twist()
    final.angular.z=0
    cmd_vel_pub.publish(final)

        
"""    while not rospy.is_shutdown():
        # Calculate the distance to the desired position
        distance = sqrt((desired_x - x) * 2 + (desired_y - y) * 2)

        # Create a Twist message with linear velocity
        cmd_vel = Twist()

        # Set linear velocity based on distance to the desired position
        if distance > 0.1:
            cmd_vel.linear.x = 0.1  # Adjust the linear velocity as needed
        else:
            cmd_vel.linear.x = 0.0  # Stop when the desired position is reached

        cmd_vel_pub.publish(cmd_vel)

        # Check if the robot is close to the desired position
        if distance < 0.1:
            print("reached the desired position!")
            #rospy.loginfo("Reached the desired position.")
            break"""


def april_tag_callback(data):
    global my_id
    if data.detections:
        detection = data.detections[0]
        try:
            my_id = detection.id[0]
        except Exception:
            pass
if __name__ == '__main__':
    try:
        move_to_desired_position(0)

    except rospy.ROSInterruptException:
        pass