#!/usr/bin/env python3

import time
import rospy

from simple_pid import PID
from std_msgs.msg import Float32, Int16
from arlo_control_msgs.msg import WheelsPower


class PidVelocity:
    """computes pid for robot as a node, needed for each wheel"""

    def __init__(self):
        rospy.init_node("pid_velocity")
        self._kp = rospy.get_param("~Kp", 10)
        self._ki = rospy.get_param("~Ki", 10)
        self._kd = rospy.get_param("~Kd", 0.0001)
        self.ouput_max = rospy.get_param("~out_max", 127)
        self.ouput_min = rospy.get_param("~out_min", -127)
        self.ticks_per_meter = rospy.get_param("ticks_meter", 20)
        self.rate = rospy.get_param("~rate", 30)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 4)

        self.cur_vel = 0
        self.target_vel = 0
        self.rate = rospy.Rate(self.rate)

        self.power_msg = Float32()
        self.wheel_vel_msg = Float32()

        self.time_tick = time.time()

        # subscribers
        rospy.Subscriber("wheel", Int16, self.wheel_callback)
        rospy.Subscriber("wheel_vtarget", Float32, self.target_callback)

        # publishers
        self.wheel_vel_pub = rospy.Publisher("wheel_vel", Float32, queue_size=10)
        self.wheel_power_pub = rospy.Publisher("motor_cmd", Float32, queue_size=10)

        self.wheels_power_pub = rospy.Publisher(
            "wheels_power", WheelsPower, queue_size=10
        )

        ## pid definition
        self.pid = PID(self._kp, self._ki, self._kd, setpoint=0)
        self.pid.output_limits = (self.ouput_min, self.ouput_max)
        self.pid.sample_time = 1 / 15
        rospy.on_shutdown(self.cleanup)
        # self.pid.proportional_on_measurement = True

    def cleanup(self):
        """stops robot movement if ros nodes are killed"""
        wheel_power_stop = WheelsPower()
        wheel_power_stop.left_power = 0
        wheel_power_stop.right_power = 0

        self.wheels_power_pub.publish(wheel_power_stop)

    def wheel_callback(self, msg):
        """gets current velocity of the wheel and saves it"""
        self.cur_vel = float(msg.data) / float(self.ticks_per_meter)

    def target_callback(self, msg):
        """changes the setpoint value"""
        self.target_vel = msg.data
        self.pid.setpoint = self.target_vel
        self.time_tick = time.time()

    def start(self):
        """starts the pid computation and control"""
        while not rospy.is_shutdown():
            if time.time() - self.time_tick > self.timeout_ticks:
                self.power_msg.data = 0
                self.wheel_power_pub.publish(self.power_msg)
            else:
                cur_output = self.pid(self.cur_vel)
                self.power_msg.data = cur_output
                self.wheel_vel_msg.data = self.cur_vel
                self.wheel_vel_pub.publish(self.wheel_vel_msg)
                self.wheel_power_pub.publish(self.power_msg)
                print("pid computed", cur_output)

            self.rate.sleep()


if __name__ == "__main__":
    pid_controller = PidVelocity()
    pid_controller.start()
