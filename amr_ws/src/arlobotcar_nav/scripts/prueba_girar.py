#! /usr/bin/env python
import pandas as pd
import time
from tutorial_files.move_to_table import move_to_goal
from tutorial_files.getPosition import get_position_by_id
from tutorial_files.new_orientation import OrientationReacher

#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int16

class Another:
    def __init__(self):
        self.is_position_reached = False
        self.timer_active = True
        self.my_id=None
        #rospy.init_node('position_reach_timer', anonymous=True)
        self.mapa = None
        self.id = None
        self.x_pos = None
        self.y_pos = None
    def callback(self,msg):
        rospy.loginfo("IN the callback")
        number = int(msg.data)
        orientation_class = OrientationReacher(number)
        orientation_class.run()
        if(orientation_class.is_position_reached):
            rospy.loginfo("nailed")

        
    def run(self):
        # Code to command the robot to move to the target position
        detection_topic = "/nodo_lectura"
        rospy.init_node('mooove', anonymous=False)

        rospy.Subscriber(detection_topic, String, self.callback)
        rate = rospy.Rate(10)  # Control loop rate (10 Hz)
        while not rospy.is_shutdown() and self.my_id == None:
            rospy.loginfo("no changues")
            rate.sleep()
        rospy.loginfo("HOLA")

if __name__ == "__main__":
    position_reach_timer = Another()
    position_reach_timer.run()
