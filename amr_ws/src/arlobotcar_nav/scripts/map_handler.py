#! /usr/bin/env python
import pandas as pd
import time
import subprocess
import rospy
from std_msgs.msg import String, Int16
import threading
class Walter_Mapping:
    def __init__(self):
        self.instruction = None
        self.map_name = None
        self.finished = False
    def run_map_launcher(self):
        package_name = "arlobotcar_nav"
        launch_file_name = "arlobotcar_map_cartographer.launch"
        command = ["roslaunch", package_name, launch_file_name]
        subprocess.run(command)
        #run_map.wait()
    def run_cartographer_finisher(self,map_name):
        package_name = "arlobotcar_nav"
        executable_name = "cartographer_finisher.py"
        command = ["rosrun", package_name, executable_name, map_name]
        variable = subprocess.run(command)
        #variable.wait()
    def callback(self,msg):
        raw_information = msg.data.split(",")
        rospy.loginfo(raw_information)
        rospy.loginfo(raw_information[0])
        rospy.loginfo(bool(raw_information[0]))
        if(raw_information[0]== "0"):
            thread1 = threading.Thread(target=self.run_map_launcher)
            thread1.start()
            #self.run_map_launcher()
        elif(raw_information[0]!="0"):
            rospy.loginfo("Else inside")
            self.map_name = raw_information[1]
            self.finished = True
            thread2 = threading.Thread(target=self.run_cartographer_finisher(self.map_name))
            #self.run_cartographer_finisher(self.map_name)
            thread2.start() 
    def run(self):
        # Code to command the robot to move to the target position
        mapping_topic = "/mapping_node"
        rospy.init_node('Mobile_Mapper', anonymous=False)
        rospy.Subscriber(mapping_topic, String, self.callback)
        rate = rospy.Rate(10)  # Control loop rate (10 Hz)
        while not rospy.is_shutdown() and self.finished == False:
            #rospy.loginfo(".")
            rate.sleep()

        rospy.loginfo("Walter Finished")

if __name__ == "__main__":
    position_reach_timer = Walter_Mapping()
    position_reach_timer.run()
