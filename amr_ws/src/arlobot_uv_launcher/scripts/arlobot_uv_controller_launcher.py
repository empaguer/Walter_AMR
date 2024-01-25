#! /usr/bin/env python3

import time
import rospy
from actionlib_msgs.msg import GoalID
import paho.mqtt.client as mqtt
from sfm_diff_drive.msg import SFMDriveAction, SFMDriveGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float64, Bool

rospy.init_node("arlobot_uv_controller_launcher")


# points to move trough people to disinfecting space
sfm_first_waypoint = SFMDriveGoal()
sfm_first_waypoint.goal.x = 0.0
sfm_first_waypoint.goal.y = 0.0
sfm_first_waypoint.goal.z = 0.0

sfm_second_waypoint = SFMDriveGoal()
sfm_second_waypoint.goal.x = 0.0
sfm_second_waypoint.goal.y = 0.0
sfm_second_waypoint.goal.z = 0.0

# defined trayetories and time
disinfecting_points = [[Point(1, 2, 0), 90], [Point(2, 2, 0), 60], [Point(4, 4, 0), 60]]

# mqtt credentials for activation
SHIFTR_TOKEN = "v4BmtoHRLVPHcfc4"
CLIENT_ID = "arlobot_uv_controller"
USUARIO = "paintbug935"
TOPIC_UV_ACTIVACION = "tiempoActivacion"
TOPIC_CARGADOR = "accionarCargador"

shiftr_mqtt = mqtt.Client(CLIENT_ID)
shiftr_mqtt.username_pw_set(USUARIO, password=SHIFTR_TOKEN)
shiftr_mqtt.connect("paintbug935.cloud.shiftr.io", port=1883)

move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

sfm_drive_client = actionlib.SimpleActionClient("sfm_drive_node", SFMDriveAction)

cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
cmd_vel_msg = Twist()
cmd_vel_pub = rospy.Publisher("/raw_cmd_vel", Twist, queue_size=10)

uv_light_state_pub = rospy.Publisher("/uv_light_state", Bool, queue_size=1)


def stop_callback(msg):
    """cancels all goals if needed according to a message received

    Args:
        msg (std_msgs.String): it is string standard of ros
    """
    if msg.data == "robot emergency cancel":
        sfm_drive_client.cancel_all_goals()
        move_base_client.cancel_all_goals()
        rospy.signal_shutdown()


def battery_voltage_callback(msg):
    """function that reacts to voltage and
    stops all actions to start auto docking

    Args:
        msg (std_msgs.Float64): simple float64 type
    """
    if msg.data <= 10.5:
        sfm_drive_client.cancel_all_goals()
        move_base_client.cancel_all_goals()
        rospy.signal_shutdown()


rospy.Subscriber("/robot_report", String, stop_callback)

rospy.Subscriber("/battery_voltage", Float64, battery_voltage_callback)

if __name__ == "__main__":
    shiftr_mqtt.publish(TOPIC_CARGADOR, str(0))
    cancel_msg = GoalID()
    cancel_move_pub.publish(cancel_msg)

    time.sleep(2)

    cmd_vel_msg.linear.x = 0.15
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0
    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)

    time.sleep(2)

    sfm_drive_client.send_goal(sfm_first_waypoint)
    wait = sfm_drive_client.wait_for_result()
    uv_state_msg = Bool()
    uv_state_msg.data = True
    uv_light_state_pub.publish(uv_state_msg)
    time.sleep(2)

    for i in disinfecting_points:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = i[0].x
        goal.target_pose.pose.position.y = i[0].y
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.pose.orientation.z = 0
        move_base_client.send_goal(goal)
        wait = move_base_client.wait_for_result()
        shiftr_mqtt.publish(TOPIC_UV_ACTIVACION, str(i[1]))
        time.sleep(i[1] + 15)
    uv_state_msg.data = False
    uv_light_state_pub.publish(uv_state_msg)
    time.sleep(2)
    sfm_drive_client.send_goal(sfm_second_waypoint)
    wait = sfm_drive_client.wait_for_result()

    time.sleep(5)
    rospy.loginfo("Desinfección terminada")
