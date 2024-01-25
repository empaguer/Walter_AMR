#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt
x=0.0
y=0.0
# Define a callback function to get the robot's current position
def odom_callback(msg):
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    print(" x Position: ",x)
    print(" Y Position: " ,y)
def move_to_desired_position():
    global x, y
    rospy.init_node('move_to_desired_position', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while x is None or y is None :
        rospy.sleep(0.1)
    # Define the desired position
    desired_x = 0.4
    desired_y = 0.0

    rate = rospy.Rate(10)  # Control loop rate (10 Hz)

    while not rospy.is_shutdown():
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
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        move_to_desired_position()
    except rospy.ROSInterruptException:
        pass