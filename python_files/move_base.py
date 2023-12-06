#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point

def move_to_goal(xGoal, yGoal, yaw_goal):
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_goal)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False

if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)

    parser = argparse.ArgumentParser(description='Move the robot to a goal (x, y, yaw).')
    parser.add_argument('x_goal', type=float, help='X coordinate of the goal')
    parser.add_argument('y_goal', type=float, help='Y coordinate of the goal')
    parser.add_argument('yaw_goal', type=float, help='Yaw angle in radians')

    args = parser.parse_args()

    rospy.loginfo("Start go to goal")
    move_to_goal(args.x_goal, args.y_goal, args.yaw_goal)
    rospy.spin()