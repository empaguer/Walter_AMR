#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point

def move_to_goal(xGoal, yGoal):
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
    
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(90))
    if(ac.get_state() == GoalStatus.SUCCEEDED):
        
        rospy.loginfo("You have reached the destination")
        ##Aqui puede ir el mandado de informacion a la esp por mensaje para poner colores
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False

