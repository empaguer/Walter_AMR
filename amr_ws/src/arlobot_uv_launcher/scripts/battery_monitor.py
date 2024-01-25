#! /usr/bin/env python3

import time
import rospy
import actionlib
from arlo_auto_docking.msg import AutoDockAction, AutoDockGoal
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import paho.mqtt.client as mqtt

rospy.init_node("battery_monitor_node")


# posicion para auto docking
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 0.0
goal.target_pose.pose.position.y = 0.0
goal.target_pose.pose.orientation.w = 1
goal.target_pose.pose.orientation.z = 0

# conexion a shiftr
# mqtt credentials for activation
SHIFTR_TOKEN = "v4BmtoHRLVPHcfc4"
CLIENT_ID = "arlobot_uv_controller"
USUARIO = "paintbug935"
TOPIC_UV_ACTIVACION = "tiempoActivacion"
TOPIC_CARGADOR = "accionarCargador"

shiftr_mqtt = mqtt.Client(CLIENT_ID)
shiftr_mqtt.username_pw_set(USUARIO, password=SHIFTR_TOKEN)
shiftr_mqtt.connect("paintbug935.cloud.shiftr.io", port=1883)


# goal de auto docking
auto_dock_goal = AutoDockGoal()
auto_dock_goal.request = "request"

move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

auto_dock_client = actionlib.SimpleActionClient("auto_dock_node", AutoDockAction)


def low_battery_callback(msg):
    """starts auto docking procedure"""
    if msg.data <= 10.5:
        time.sleep(5)
        move_base_client.send_goal(goal)
        wait = move_base_client.wait_for_result()
        time.sleep(5)
        auto_dock_client.send_goal(auto_dock_goal)
        wait = auto_dock_client.wait_for_result()
        time.sleep(5)
        shiftr_mqtt.publish(TOPIC_CARGADOR, str(1))
        rospy.loginfo("Proceso de auto docking terminado")


rospy.Subscriber("/battery_voltage", Float32, low_battery_callback)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.spin()
