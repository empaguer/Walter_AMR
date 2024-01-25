#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist

# Global variables to store time and angular velocity data
times = []
angular_velocities = []

# Global variable to store the last received cmd_vel message
last_cmd_vel_msg = None

def cmd_vel_callback(msg):
    global last_cmd_vel_msg
    last_cmd_vel_msg = msg

def plot_cmd_vel():
    rospy.init_node('cmd_vel_plotter', anonymous=True)

    # Initialize the live plot
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.set_title('Angular Velocity vs. Time')
    ax.grid(True)

    # Subscribe to the "cmd_vel" topic, expecting messages of type Twist
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    rate = rospy.Rate(10)  # Set the loop rate to 10 Hz (adjust as needed)
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if last_cmd_vel_msg is not None:
            # Record the current time
            current_time = rospy.Time.now().to_sec()

            # Extract the angular velocity (angular.z) from the last received cmd_vel message
            angular_velocity = last_cmd_vel_msg.angular.z
            elapsed_time = current_time - start_time
            # Append the data to the lists
            times.append(elapsed_time)
            angular_velocities.append(angular_velocity)

            # Update the live plot
            line.set_data(times, angular_velocities)
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