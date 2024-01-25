#! /usr/bin/env python
import pandas as pd
import time
from tutorial_files.move_to_table import move_to_goal
from tutorial_files.getPosition import get_position_by_id

#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class OrientationReacher:
    def __init__(self):
        self.is_position_reached = False
        self.timer_active = True
        self.my_id=999
        #rospy.init_node('position_reach_timer', anonymous=True)
        self.mapa = None
        self.id = None
        self.x_pos = None
        self.y_pos = None
        self.validate = False
    def callback(self,msg):
        rospy.loginfo("IN the callback")
        rospy.loginfo(msg)
        self.mapa = msg.data.split(",")[0]
        self.id = int(msg.data.split(",")[1])
        rospy.loginfo(self.mapa)
        rospy.loginfo(self.id)
        self.x_pos, self.y_pos = get_position_by_id(self.mapa, self.id)
        rospy.loginfo(self.x_pos)
        rospy.loginfo(self.y_pos)
        validacion =move_to_goal(self.x_pos,self.y_pos)
        if(validacion):
            self.validate = True
        #if(validacion):
        #    rospy.loginfo("nailed")
        #else:
        #    rospy.loginfo("failed")
        
    def run(self):
        # Code to command the robot to move to the target position
        detection_topic = "/navigation_node"
        rospy.init_node('mooove', anonymous=False)

        rospy.Subscriber(detection_topic, String, self.callback)
        rate = rospy.Rate(10)  # Control loop rate (10 Hz)
        while not rospy.is_shutdown() and not self.validate:
            #rospy.loginfo("no changues")
            rate.sleep()
        rospy.loginfo("HOLA")

if __name__ == "__main__":
    position_reach_timer = OrientationReacher()
    position_reach_timer.run()
