#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped

# Global variables to store time and torque data
times = []
torque_x = []
torque_y = []
torque_z = []

# Global variable to store the last received WrenchStamped message
last_ft_sensor_msg = None

def ft_sensor_callback(msg):
    global last_ft_sensor_msg
    last_ft_sensor_msg = msg

def plot_ft_sensor():
    rospy.init_node('ft_sensor_plotter', anonymous=True)

    # Initialize the live plot
    plt.ion()
    fig, ax = plt.subplots()
    line_x, = ax.plot([], [], label='Torque X')
    line_y, = ax.plot([], [], label='Torque Y')
    line_z, = ax.plot([], [], label='Torque Z')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (N.m)')
    ax.set_title('Torque vs. Time')
    ax.grid(True)
    ax.legend()

    # Subscribe to the "/ft_sensor_llanta_izquierda" topic, expecting messages of type WrenchStamped
    rospy.Subscriber("/ft_sensor_llanta_izquierda", WrenchStamped, ft_sensor_callback)

    rate = rospy.Rate(10)  # Set the loop rate to 10 Hz (adjust as needed)

    while not rospy.is_shutdown():
        if last_ft_sensor_msg is not None:
            # Record the current time
            current_time = rospy.Time.now().to_sec()

            # Extract the torque components (torque.x, torque.y, torque.z) from the last received ft_sensor message
            torque_x.append(last_ft_sensor_msg.wrench.torque.x)
            torque_y.append(last_ft_sensor_msg.wrench.torque.y)
            torque_z.append(last_ft_sensor_msg.wrench.torque.z)
            times.append(current_time)

            # Update the live plot
            line_x.set_data(times, torque_x)
            line_y.set_data(times, torque_y)
            line_z.set_data(times, torque_z)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)  # Pause to allow the plot to update

        rate.sleep()  # Sleep to control the loop rate

if __name__ == '__main__':
    try:
        plot_ft_sensor()
    except rospy.ROSInterruptException:
        pass