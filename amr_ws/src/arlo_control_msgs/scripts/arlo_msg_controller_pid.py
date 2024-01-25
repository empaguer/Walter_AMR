#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32
from arlo_control_msgs.msg import WheelsEncoders, WheelsPower



class ArloMsgController:
    """Helps to transmit velocity and encoders values less effortlessly
    """
    def __init__(self):
        rospy.init_node("arlo_msg_control_node")

        # variables
        # power
        self.left_power = 0
        self.right_power = 0
        self.last_left_power = 0
        self.last_right_power = 0
        self.power_msg = WheelsPower()

        # encoders
        self.left_encoder_msg = Int16()
        self.right_encoder_msg = Int16()

        self.left_velocity_msg = Int16()
        self.right_velocity_msg = Int16()

        self.rate = rospy.Rate(50)

        # subscribers
        # power
        self.left_power_sub = rospy.Subscriber(
            "/lmotor_cmd", Float32, self.left_power_callback
        )

        self.right_power_sub = rospy.Subscriber(
            "/rmotor_cmd", Float32, self.right_power_callback
        )

        # encoders
        self.encoders_sub = rospy.Subscriber(
            "/wheels_encoder", WheelsEncoders, self.encoders_callback
        )

        self.wheel_velocity_sub = rospy.Subscriber(
            "/wheels_vel", WheelsEncoders, self.wheel_vel_callback
        )

        # publishers
        # power
        self.power_pub = rospy.Publisher("/wheels_power", WheelsPower, queue_size=10)

        # encoders
        self.left_encoder_pub = rospy.Publisher("/lwheel", Int16, queue_size=10)
        self.right_encoder_pub = rospy.Publisher("/rwheel", Int16, queue_size=10)

        # velocities
        self.left_vel_pub = rospy.Publisher("/lwheel_vel_tick", Int16, queue_size=10)
        self.right_vel_pub = rospy.Publisher("/rwheel_vel_tick", Int16, queue_size=10)

    # callbacks
    # power
    def left_power_callback(self, msg):
        """gets power for left wheel and saves it

        Args:
            msg (std_msgs.Int16): just an int
        """
        self.left_power = int(msg.data)
        # print("left power:", self.left_power)

    def right_power_callback(self, msg):
        """gets power for right wheel and saves it

        Args:
            msg (std_msgs.Int16): just an int
        """
        self.right_power = int(msg.data)

    # wheel velocity
    def wheel_vel_callback(self, msg):
        """gets wheels velocities and stores them, coming from esp32

        Args:
            msg (arlo_control_msgs.WheelsEncoders): two fields with int16 value type
            used to send encoder ticks per second
        """
        self.left_velocity_msg.data = msg.left_encoder
        self.right_velocity_msg.data = msg.right_encoder

    # encoders
    def encoders_callback(self, msg):
        """receives encoders values and stores then

        Args:
            msg (arlo_control_msgs.WheelsEncoders): two fields with int16 value type
        """
        self.left_encoder_msg.data = msg.left_encoder
        self.right_encoder_msg.data = msg.right_encoder

    def start(self):
        """starts the process of manging the messages
        """
        while not rospy.is_shutdown():
            if (
                self.last_left_power != self.left_power
                or self.last_right_power != self.right_power
            ):
                self.power_msg.left_power = float(self.left_power)
                self.power_msg.right_power = float(self.right_power)
                self.last_left_power = int(self.left_power)
                self.last_right_power = int(self.right_power)
                self.power_pub.publish(self.power_msg)
            self.left_encoder_pub.publish(self.left_encoder_msg)
            self.right_encoder_pub.publish(self.right_encoder_msg)
            self.left_vel_pub.publish(self.left_velocity_msg)
            self.right_vel_pub.publish(self.right_velocity_msg)

            self.rate.sleep()


if __name__ == "__main__":
    arlo_control = ArloMsgController()
    arlo_control.start()
