#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int16
from .arlorobot import ArloRobot

right_speed = 0
left_speed = 0
LEFT_ENCODER_TOPIC = "/lwheel"
RIGHT_ENCODER_TOPIC = "/rwheel"
LEFT_SPEED_TOPIC = "/lwheel_vtarget"
RIGHT_SPEED_TOPIC = "/rwheel_vtarget"
RESET_COMS_TOPIC = "/arlo_reset_com"
TICKS_METER = 300
arlobot = ArloRobot(
    serial_id=rospy.get_param("serial_port", "/dev/ttyUSB1"), baudrate=19200
)
arlobot.set_baud_rate(115200, True)
try:
    arlobot.close()
except:
    pass
arlobot = ArloRobot(
    serial_id=rospy.get_param("serial_port", "/dev/ttyUSB1"),
    baudrate=rospy.get_param("serial_baudrate", 115200),
)
arlobot.clear_counts(True)


def left_speed_callback(msg):
    global left_speed
    speed = msg.data
    speed = speed * TICKS_METER
    # print('left speed', speed)
    tlspeed = int(speed)
    left_speed = tlspeed


def right_speed_callback(msg):
    global right_speed
    speed = msg.data
    speed = speed * TICKS_METER
    # print('right speed', speed)
    trspeed = int(speed)
    right_speed = trspeed


def reset_coms_callback(msg):
    global arlobot
    arlobot = ArloRobot(
        serial_id=rospy.get_param("serial_port", "/dev/ttyUSB1"), baudrate=19200
    )
    arlobot.set_baud_rate(115200, True)
    try:
        arlobot.close()
    except:
        pass
    arlobot = ArloRobot(
        serial_id=rospy.get_param("serial_port", "/dev/ttyUSB1"),
        baudrate=rospy.get_param("serial_baudrate", 115200),
    )


def send_speed(lspeed=0, rspeed=0):
    if lspeed == 0 and rspeed == 0:
        arlobot.go_speed(0, 0, True)
        arlobot.go_speed(0, 0, True)
    else:
        arlobot.go_speed(lspeed, rspeed, True)


def clean_up():
    try:
        arlobot.go_speed(0, 0, True)
        arlobot.go_speed(0, 0, True)
        arlobot.close()
    except:
        pass


def listener():
    global left_speed
    global right_speed
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("encoders", anonymous=True)
    rospy.on_shutdown(clean_up)

    rospy.Subscriber(LEFT_SPEED_TOPIC, Float32, left_speed_callback, queue_size=1)
    rospy.Subscriber(RIGHT_SPEED_TOPIC, Float32, right_speed_callback, queue_size=1)
    rospy.Subscriber(RESET_COMS_TOPIC, Float32, reset_coms_callback, queue_size=1)
    rpub = rospy.Publisher(RIGHT_ENCODER_TOPIC, Int16, queue_size=5)
    lpub = rospy.Publisher(LEFT_ENCODER_TOPIC, Int16, queue_size=5)
    rate = rospy.Rate(15)  # 30hz
    while not rospy.is_shutdown():
        counts = arlobot.read_counts()
        send_speed(left_speed, right_speed)
        if isinstance(counts, list) and len(counts) == 2:
            left = counts[0]
            right = counts[1]
            if isinstance(left, int) and isinstance(right, int):
                rpub.publish(right)
                lpub.publish(left)
        rate.sleep()
    arlobot.go_speed(0, 0, True)
    arlobot.close()


if __name__ == "__main__":
    listener()
