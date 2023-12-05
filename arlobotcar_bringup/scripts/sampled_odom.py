#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster
import math
class RobotBase:
    def __init__(self):
        rospy.init_node('my_odom_publiser', anonymous=True)

        self.linear_velocity_x_ = 0.0
        self.linear_velocity_y_ = 0.0
        self.angular_velocity_z_ = 0.0
        self.last_vel_time_ = rospy.Time.now()
        self.vel_dt_ = 0.0
        self.x_pos_ = 0.0
        self.y_pos_ = 0.0
        self.heading_ = 0.0
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.quaternion = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        self.linear_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.angular_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.pose_covariance = [0.0] * 36  # 6x6 matrix
        self.twist_covariance = [0.0] * 36  # 6x6 matrix
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform

        self.linear_scale_x = rospy.get_param('~linear_scale_x', 1.0)
        self.odom_frame = rospy.get_param('~odom_frame', 'camera_odom_frame')
        self.base_footprint_frame = rospy.get_param('~base_footprint_frame', 'base_footprint')

        self.velocity_subscriber_ = rospy.Subscriber("/camera/odom/sample", Odometry, self.odom_callback)
        self.odom_publisher_ = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odomBroadcaster = TransformBroadcaster()

    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def odom_callback(self,odom_msg):
        self.position = {'x': odom_msg.pose.pose.position.x, 
                         'y': odom_msg.pose.pose.position.y, 
                         'z': odom_msg.pose.pose.position.z}

        self.quaternion = {'x': odom_msg.pose.pose.orientation.x,
                           'y': odom_msg.pose.pose.orientation.y,
                           'z': odom_msg.pose.pose.orientation.z,
                           'w': odom_msg.pose.pose.orientation.w}

        self.linear_velocity = {'x': odom_msg.twist.twist.linear.x,
                                'y': odom_msg.twist.twist.linear.y,
                                'z': odom_msg.twist.twist.linear.z}

        self.angular_velocity = {'x': odom_msg.twist.twist.angular.x,
                                 'y': odom_msg.twist.twist.angular.y,
                                 'z': odom_msg.twist.twist.angular.z}
        self.pose_covariance = list(odom_msg.pose.covariance)
        self.twist_covariance = list(odom_msg.twist.covariance)
        
    def update(self):
        current_time = rospy.Time.now()


        # Create quaternion from yaw

        self.odomBroadcaster.sendTransform(
            (self.position['x'], self.position['y'], self.position['z']),
            (self.quaternion['x'], self.quaternion['y'],self.quaternion['z'], self.quaternion['w']),
            rospy.Time.now(),
            self.base_footprint_frame,
            self.odom_frame 
            )
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame

        # Position and orientation
        odom.pose.pose.position.x = self.position['x']
        odom.pose.pose.position.y = self.position['y']
        odom.pose.pose.position.z = self.position['z']
        odom.pose.pose.orientation.x = self.quaternion['x']
        odom.pose.pose.orientation.y = self.quaternion['y']
        odom.pose.pose.orientation.z = self.quaternion['z']
        odom.pose.pose.orientation.w = self.quaternion['w']

        # Pose covariance
        odom.pose.covariance = self.pose_covariance

        # Velocity
        odom.twist.twist.linear.x = self.linear_velocity['x']
        odom.twist.twist.linear.y = self.linear_velocity['y']
        odom.twist.twist.linear.z = self.linear_velocity['z']
        odom.twist.twist.angular.x = self.angular_velocity['x']
        odom.twist.twist.angular.y = self.angular_velocity['y']
        odom.twist.twist.angular.z = self.angular_velocity['z']

        # Twist covariance
        odom.twist.covariance = self.twist_covariance

        # Publish the modified odometry message
        self.odom_publisher_.publish(odom)
if __name__ == '__main__':
    try:
        robot_base = RobotBase()
        robot_base.spin()
    except rospy.ROSInterruptException:
        pass
