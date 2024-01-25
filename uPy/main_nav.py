"""Nos permite hacer control del robot de manera autonoma
enviando datos de encoder sin pid
"""

import uros
import utime
from std_msgs import Float32, Int16  # rosserial messages
from arlorobot import ArloRobot


class ArlorobotMove:
    """Esta clase maneja dos threads que publican datos del encoder
    y subcriben a datos de velocidad
    """

    def __init__(self):

        # constants
        self.left_encoder_topic = "/lwheel"
        self.right_encoder_topic = "/rwheel"
        self.left_speed_topic = "/lwheel_vtarget"
        self.right_speed_topic = "/rwheel_vtarget"
        self.ticks_meter = 300

        self.arlobot = ArloRobot(serialid=2)  # serial=2, tx=17, rx=16, baudrate=115200
        self.node = uros.NodeHandle(0, 115200)
        self.left_encoder = Int16()
        self.right_encoder = Int16()
        self.left_speed = 0
        self.right_speed = 0

    def left_speed_callback(self, msg):
        """funcion que recibe la velocidad de las llantas
        y envia el comando al driver para asignarle la velocidad

        Args:
            msg (std_msgs.Float32): es un tipo de datos standard de ROS del tipo float32
        """
        print("velocidad llanta izquierda", int(msg.data))
        self.left_speed = self.ticks_meter * int(msg.data)

    def right_speed_callback(self, msg):
        """funcion para recibir dato de velocidad de llanta derecha
        y mandarlo al driver

        Args:
            msg (std_msgs.Float32): es un tipo de datos standard de ROS del tipo float32
        """
        print("velocidad llanta derecha", int(msg.data))
        self.right_speed = self.ticks_meter * int(msg.data)

    def start(self):
        """this func starts the threads"""
        self.node.subscribe(self.left_speed_topic, Float32, self.left_speed_callback)
        self.node.subscribe(self.right_speed_topic, Float32, self.right_speed_callback)
        while True:
            self.arlobot.go_speed(self.left_speed, self.right_speed)
            utime.sleep(0.5)
            try:
                left = int(
                    self.arlobot.read_left_counts()
                )  # encoders read from arlorobot
                right = int(self.arlobot.read_right_counts())
                if isinstance(left, int) and isinstance(right, int):
                    print("encoder izquierdo", left)
                    print("encoder derecho", right)
                    self.left_encoder.data = left  # messages defined
                    self.right_encoder.data = right
                    self.node.publish(
                        self.left_encoder_topic, self.left_encoder
                    )  # encoders value published
                    self.node.publish(self.right_encoder_topic, self.right_encoder)
            except ValueError:
                print("encoder error bad message")


my_arlobot = ArlorobotMove()
my_arlobot.start()
