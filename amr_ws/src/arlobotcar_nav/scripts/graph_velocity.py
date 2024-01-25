#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
# Global variables to store time and angular velocity data
times = []
input_velocities = []
esp_velocities = []
# Global variable to store the last received cmd_vel message
last_cmd_vel_msg = None
last_esp_vel_msg = None
def cmd_vel_callback(msg):
    global last_cmd_vel_msg
    last_cmd_vel_msg = msg
def esp_vel_callback(msg):
    global last_esp_vel_msg
    last_esp_vel_msg = msg
def plot_cmd_vel():
    rospy.init_node('PID_plotter', anonymous=True)

    # Initialize the live plot
    plt.ion()
    fig, ax = plt.subplots()
    line1, = ax.plot([], [])
    line1.set_label("Input cmd velocity")
    line2, =  ax.plot([], [])
    line2.set_label("Walter Real Velocity")
    line2.set_linestyle('--')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Linear Velocity (m/s)')
    ax.set_title('Linear Velocity Controller vs. Time')
    ax.legend(["Input Velocity", "Walter Velocity"])
    ax.grid(True)

    # Subscribe to the "cmd_vel" topic, expecting messages of type Twist
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("esp_velocity", Float32, esp_vel_callback)
    rate = rospy.Rate(10)  # Set the loop rate to 10 Hz (adjust as needed)
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if last_cmd_vel_msg is not None:
            # Record the current time
            current_time = rospy.Time.now().to_sec()

            # Extract the angular velocity (angular.z) from the last received cmd_vel message
            cmd_velocity = last_cmd_vel_msg.linear.x
            esp_velocity = last_esp_vel_msg.data
            elapsed_time = current_time - start_time
            # Append the data to the lists
            times.append(elapsed_time)
            input_velocities.append(cmd_velocity)
            esp_velocities.append(esp_velocity)
            # Update the live plot
            line1.set_data(times, input_velocities)
            line2.set_data(times,esp_velocities)
            ax.set_ylim(0,0.5)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)  # Pause to allow the plot to update

        rate.sleep()  # Sleep to control the loop rate

if __name__ == '__main__':
    try:
        plot_cmd_vel()
    except rospy.ROSInterruptException:
        pass