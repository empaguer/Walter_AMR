#! /usr/bin/env python3

import time
from actionlib_msgs.msg import GoalID
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

rospy.init_node("people_control_node", anonymous=True)

SHIFTR_TOKEN = "v4BmtoHRLVPHcfc4"
CLIENT_ID = "people_control_node"
USUARIO = "paintbug935"
TOPIC_STOP = "stop"

# topics
PEOPLE_PRESENT_TOPIC = "/presence"
VEL_TOPIC = "/twist"

# twist pub
twist_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
twist_msg = Twist()
twist_msg.linear.x = 0
twist_msg.angular.x = 0
twist_msg.angular.y = 0
twist_msg.angular.z = 0

uv_state = False

# cancel move base pub
cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
cancel_msg = GoalID()

robot_reporter_pub = rospy.Publisher("/robot_report", String, queue_size=1)


def uv_state_callback(msg):
    """receives value to change uv state"""
    global uv_state
    uv_state = msg.data


rospy.Subscriber("/uv_light_state", Bool, uv_state_callback)


def cancel_nav():
    """cancels all navigations tasks"""
    cancel_move_pub.publish(cancel_msg)
    robot_report_message = String()
    robot_report_message.data = "robot emergency cancel"
    robot_reporter_pub.publish(robot_report_message)
    time.sleep(1)
    twist_pub.publish(twist_msg)


def people_present_callback(data):
    """cancels all navigation if people is present

    Args:
        data (std_msgs.Bool): simple boolean from ros
    """
    global uv_state
    if uv_state:
        msg = data.data
        if msg:
            cancel_nav()
        else:
            pass
    else:
        pass


def on_message(client, userdata, message):
    """cancels navigation if message is 'stop'

    Args:
        client (managed by module): managed by module
        userdata (managed by module): managed by module
        message (managed by module): managed by module
    """
    data = str(message.payload.decode("utf-8"))
    if data == "stop":
        cancel_nav()


shiftr_mqtt = mqtt.Client(CLIENT_ID)
shiftr_mqtt.username_pw_set(USUARIO, password=SHIFTR_TOKEN)
shiftr_mqtt.connect("paintbug935.cloud.shiftr.io", port=1883)

shiftr_mqtt.on_message = on_message
shiftr_mqtt.subscribe(TOPIC_STOP, qos=0)
shiftr_mqtt.loop_start()


people_present_sub = rospy.Subscriber(
    PEOPLE_PRESENT_TOPIC, Bool, people_present_callback
)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        time.sleep(0.001)
