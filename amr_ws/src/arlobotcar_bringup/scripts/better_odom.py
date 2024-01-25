#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class WheelOdometryPublisher:
    def __init__(self):
        rospy.init_node('wheel_odometry_publisher')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.angular_velocity_z_ = 0.0
        self.last_time = rospy.Time.now()

        # Subscriber for cmd_vel
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Publisher for odometry
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster()

    def cmd_vel_callback(self, twist):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Update pose based on velocity commands (simple integration)
        self.x += twist.linear.x * dt
        self.y += twist.linear.y * dt

        # Update orientation based on angular velocity
        self.theta += twist.angular.z * dt
        self.angular_velocity_z_=twist.angular.z
        # Publish Odometry and Broadcast Transform
        self.publish_odometry(current_time)
        self.broadcast_transform()
        # Update last_time
        self.last_time = current_time

    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Populate the pose information
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Convert yaw angle to quaternion
        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Populate the twist information
        odom.twist.twist.linear.x = 0.0  # Assuming no linear velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = twist.angular.z  # Use the received angular velocity

        # Publish odometry
        self.odom_pub.publish(odom)

    def broadcast_transform(self):
        # Broadcast the transform from "odom" to "base_link"
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quaternion_from_euler(0, 0, self.theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        odometry_publisher = WheelOdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
