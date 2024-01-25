#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
class OrientationReacher:
    def __init__(self,desired_id):
        self.is_position_reached = False
        self.timer_active = True
        self.my_id=999
        self.desired_id=desired_id
        #rospy.init_node('position_reach_timer', anonymous=True)
        self.timer = rospy.Timer(rospy.Duration(30), self.timer_callback, oneshot=True)

    def timer_callback(self, event):
        if not self.is_position_reached:
            rospy.loginfo("30 seconds have passed, aborting the task.")
            # Code to abort the task
            self.timer_active = False

    def position_check_function(self):
        if(self.my_id==self.desired_id):
            rospy.loginfo("ID FOUND, STOPPING WALTER")
            self.is_position_reached = True
        # Code to check if the position is reached
        # if position is reached:
        #     self.is_position_reached = True
        

    def run(self):
        # Code to command the robot to move to the target position
        detection_topic = "/tag_detections"
        rospy.Subscriber(detection_topic, AprilTagDetectionArray, self.april_tag_callback)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)  # Control loop rate (10 Hz)
        while not rospy.is_shutdown() and self.timer_active:
            #rospy.loginfo("inside while")
            cmd_vel = Twist()
            self.position_check_function()
            #rospy.loginfo("not here")
            if self.is_position_reached:
                cmd_vel.angular.z= 0
                cmd_vel_pub.publish(cmd_vel)
                break
            else:
                #rospy.loginfo("spinning")
                cmd_vel.angular.z= 0.25                
                cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
        # Stop the timer if it's still running
        if self.timer_active:
            #closing
            cmd_vel.angular.z= 0
            cmd_vel_pub.publish(cmd_vel)
            self.timer.shutdown()

        # Additional code for cleanup or further actions
    def april_tag_callback(self,data):
        if data.detections:
            detection = data.detections[0]
            try:
                self.my_id = detection.id[0]
            except Exception:
                pass

if __name__ == "__main__":
    position_reach_timer = OrientationReacher()
    position_reach_timer.run()
