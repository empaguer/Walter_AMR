#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from arlo_control_msgs.msg import WheelsEncoders


class ArloMsgController:
    """manages robot velocity communication without using pid"""

    def __init__(self):
        rospy.init_node("arlo_msg_control_node")

        # variables
        # velocity

        self.rate = rospy.Rate(30)
        self.ticks_meter = 300

        self.wheel_vel = WheelsEncoders()

        self.last_left_vel = 0
        self.last_right_vel = 0

        self.cur_left_vel = 0
        self.cur_right_vel = 0

        # subscribers
        # velocity
        self.left_wheel_sub = rospy.Subscriber(
            "/lwheel_vtarget", Float32, self.left_vel_callback
        )

        self.right_wheel_sub = rospy.Subscriber(
            "/rwheel_vtarget", Float32, self.right_vel_callback
        )

        # publishers
        # power
        self.vel_pub = rospy.Publisher("/wheels_vel", WheelsEncoders, queue_size=10)

    # callbacks
    # power
    def left_vel_callback(self, msg):
        """gets left wheel velocity and saves it"""
        self.cur_left_vel = int(msg.data * 300)

    def right_vel_callback(self, msg):
        """gets right wheel velocity and saves it"""
        self.cur_right_vel = int(msg.data * 300)

    def start(self):
        """starts robot movement velocities management"""
        while not rospy.is_shutdown():
            self.wheel_vel.left_encoder = int(self.cur_left_vel)
            self.wheel_vel.right_encoder = int(self.cur_right_vel)
            self.last_left_vel = int(self.cur_left_vel)
            self.last_right_vel = int(self.cur_right_vel)
            self.vel_pub.publish(self.wheel_vel)

            self.rate.sleep()


if __name__ == "__main__":
    arlo_control = ArloMsgController()
    arlo_control.start()
