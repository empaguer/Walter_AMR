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
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        #rospy.loginfo("entro")
        self.linear_scale_x = rospy.get_param('~linear_scale_x', 1.0)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_footprint_frame = rospy.get_param('~base_footprint_frame', 'base_footprint')

        #self.velocity_subscriber_ = rospy.Subscriber("/sub_vel", Twist, self.vel_callback)
        self.odom_subscriber_ = rospy.Subscriber("/camera/odom/sample", Odometry, self.odom_callback)
        self.odom_publisher_ = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odomBroadcaster = TransformBroadcaster()

    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            #self.update()
            r.sleep()
    
    def odom_callback(self,msg):

        current_time = rospy.Time.now()
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame
        # Compute odometry
        self.x_pos_ = msg.pose.pose.position.x
        self.y_pos_ = msg.pose.pose.position.y
        self.z_pos_ = 0.0

        # Create quaternion from yaw
        odom_quat_z = msg.pose.pose.orientation.z
        odom_quat_w = msg.pose.pose.orientation.w
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = odom_quat_z
        quaternion.w = odom_quat_w
        self.odomBroadcaster.sendTransform(
            (self.x_pos_, self.y_pos_, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w ),
            rospy.Time.now(),
            self.base_footprint_frame,
            self.odom_frame 
            )
        
        # Set position and orientation
        odom.pose.pose.position.x = self.x_pos_
        odom.pose.pose.position.y = self.y_pos_
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion

        # Set covariance (example values, adjust as needed)
        '''
        odom.pose.covariance = [0.001, 0, 0, 0, 0, 0,
                                0, 0.001, 0, 0, 0, 0,
                                0, 0, 0.001, 0, 0, 0,
                                0, 0, 0, 0.001, 0, 0,
                                0, 0, 0, 0, 0.001, 0,
                                0, 0, 0, 0, 0, 0.001]
        '''

        odom.pose.covariance = msg.pose.covariance
        # Set velocity
        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = msg.twist.twist.linear.y
        odom.twist.twist.angular.z = msg.twist.twist.angular.z
        
        # Set twist covariance (example values, adjust as needed)
        '''
        odom.twist.covariance = [0.0001, 0, 0, 0, 0, 0,
                                 0, 0.0001, 0, 0, 0, 0,
                                 0, 0, 0.0001, 0, 0, 0,
                                 0, 0, 0, 0.0001, 0, 0,
                                 0, 0, 0, 0, 0.0001, 0,
                                 0, 0, 0, 0, 0, 0.0001]
        '''
        odom.twist.covariance =msg.twist.covariance

        self.odom_publisher_.publish(odom)
        #rospy.loginfo("entro")
    '''
    def update(self,Odometry):
        current_time = rospy.Time.now()
        #self.vel_dt_ = (current_time - self.last_vel_time_).to_sec()
        #self.last_vel_time_ = current_time

        # Compute odometry
        delta_heading = self.angular_velocity_z_ * self.vel_dt_
        delta_x = (self.linear_velocity_x_ * math.cos(self.heading_) ) * self.vel_dt_
        delta_y = (self.linear_velocity_x_ * math.sin(self.heading_) ) * self.vel_dt_

        # Update position
        self.x_pos_ += delta_x
        self.y_pos_ += delta_y
        self.heading_ += delta_heading

        # Create quaternion from yaw
        odom_quat = Quaternion(*quaternion_from_euler(0, 0, self.heading_))
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin( self.heading_ / 2 )
        quaternion.w = math.cos( self.heading_ / 2 )
        self.odomBroadcaster.sendTransform(
            (self.x_pos_, self.y_pos_, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w ),
            rospy.Time.now(),
            self.base_footprint_frame,
            self.odom_frame 
            )
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame

        # Set position and orientation
        odom.pose.pose.position.x = self.x_pos_
        odom.pose.pose.position.y = self.y_pos_
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion

        # Set covariance (example values, adjust as needed)
        odom.pose.covariance = [0.001, 0, 0, 0, 0, 0,
                                0, 0.001, 0, 0, 0, 0,
                                0, 0, 0.001, 0, 0, 0,
                                0, 0, 0, 0.001, 0, 0,
                                0, 0, 0, 0, 0.001, 0,
                                0, 0, 0, 0, 0, 0.001]

        # Set velocity
        odom.twist.twist.linear.x = self.linear_velocity_x_
        odom.twist.twist.linear.y = self.linear_velocity_y_
        odom.twist.twist.angular.z = self.angular_velocity_z_

        # Set twist covariance (example values, adjust as needed)
        odom.twist.covariance = [0.0001, 0, 0, 0, 0, 0,
                                 0, 0.0001, 0, 0, 0, 0,
                                 0, 0, 0.0001, 0, 0, 0,
                                 0, 0, 0, 0.0001, 0, 0,
                                 0, 0, 0, 0, 0.0001, 0,
                                 0, 0, 0, 0, 0, 0.0001]
        self.odom_publisher_.publish(odom)
        '''
if __name__ == '__main__':
    try:
        robot_base = RobotBase()
        robot_base.spin()
    except rospy.ROSInterruptException:
        pass
