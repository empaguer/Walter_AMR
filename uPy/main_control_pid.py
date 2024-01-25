"""main control of robot movement and function by using pid
"""

import gc
import _thread
import uros
import utime
from machine import Pin, ADC
import network
from umqtt.robust2 import MQTTClient
import machine
from std_msgs import Float32, Bool
from arlo_control_msgs import WheelsEncoders, WheelsPower  # rosserial messages
from arlorobot import ArloRobot

gc.enable()


def do_connect():
    """conecta el esp32 a wifi"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect("NETLIFE-Silva", "SASM3141")
        while not wlan.isconnected():
            pass
    # print("network config:", wlan.ifconfig())


do_connect()

utime.sleep(20)


class ArlorobotMove:
    """controls all movement and functionality of the robot"""

    def __init__(self):

        # analog read
        self.voltage_read = ADC(Pin(34))
        self.voltage_read.atten(ADC.ATTN_11DB)

        # mqtt variables light control
        self.l_uvc = Pin(15, Pin.OUT)  # 15 uv
        self.l_red = Pin(18, Pin.OUT)  #  18
        self.l_green = Pin(25, Pin.OUT)
        self.l_blue = Pin(22, Pin.OUT)  # 22
        self.message = ""
        self.is_waiting = False

        self.time_to_work = None
        self.callback_time = 0

        self.connect_sub = 0
        self.mqtt_client_error = False
        self.mqtt_reconnecting = False

        self.initial_status_lights()
        # utime.sleep(1)

        # conexion a broker

        self.shiftr_token = b"v4BmtoHRLVPHcfc4"
        self.client_id = b"ESP32"
        self.usuario_ = b"paintbug935"
        self.topic_time = b"tiempoActivacion"
        self.topic_stop = b"stop"

        self.client = MQTTClient(
            self.client_id,
            b"paintbug935.cloud.shiftr.io",
            1883,
            user=self.usuario_,
            password=self.shiftr_token,
        )  # creacion de objeto

        self.client.DEBUG = True
        # Information whether we store unsent messages with the flag QoS==0 in the queue.
        self.client.KEEP_QOS0 = False
        self.client.NO_QUEUE_DUPS = True
        # Limit the number of unsent messages in the queue.
        self.client.MSG_QUEUE_MAX = 1

        self.client.set_callback(self.uv_light_callback)
        self.client.connect(clean_session=False)  # conexion a broker
        self.client.subscribe(self.topic_time)
        self.client.subscribe(self.topic_stop)

        utime.sleep(1)

        self.timer = machine.Timer(0)
        self.timer.init(
            period=60000,
            mode=machine.Timer.PERIODIC,
            callback=self.battery_voltage_callback,
        )

        # constants
        # self.WHEEL_ENCODER_TOPIC = "/wheels_encoder"
        self.wheel_power_topic = "/wheels_power"
        self.wheel_velocity_topic = "/wheels_vel"
        self.battery_topic = "/battery_voltage"
        self.presence_stop_topic = "/presence"
        self.pir_value_topic = "/pir_value"

        self.arlobot = ArloRobot(serial_id=2, baudrate=19200, tx=17, rx=16, pace=2)
        self.node = uros.NodeHandle(1, 115200, tx=1, rx=3)
        # self.wheels_encoders = WheelsEncoders()
        self.wheels_velocity = WheelsEncoders()
        self.left_power = 0
        self.right_power = 0

        self.last_left_power = 0
        self.last_right_power = 0

        self.pir_value_msg = Bool()

        self.battery_voltage_msg = Float32()

        self.pir_pin = machine.Pin(35, machine.Pin.IN)

        _thread.start_new_thread(self.mqtt_handler, ())

    # funciones de control de luces
    def initial_status_lights(self):
        """controla el estado inicial de las luces"""
        if not self.is_waiting:
            self.l_uvc.value(1)
            self.l_red.value(1)
            self.l_green.value(0)
            self.l_blue.value(1)

    def last_status_lights(self):
        """sets last status of rgb lights"""
        self.is_waiting = True
        cur_time = utime.ticks_ms()
        while utime.ticks_ms() - cur_time < 4000:
            self.l_uvc.value(1)
            self.l_red.value(0)
            self.l_green.value(0)
            self.l_blue.value(1)
        self.l_uvc.value(1)
        self.l_red.value(1)
        self.l_green.value(0)
        self.l_blue.value(1)
        self.is_waiting = False

    def light_controller_active(self):
        """activates rgb lights and uvc"""
        self.l_uvc.value(0)
        self.l_red.value(0)
        self.l_green.value(1)
        self.l_blue.value(1)

    # callbacks

    def uv_light_callback(self, topic, msg, retained, duplicate):
        """function to control uv light according to message received by mqtt

        Args:
            topic (managed by module): managed by module
            msg (managed by module): managed by module
            retained (managed by module): managed by module
            duplicate (managed by module): managed by module
        """
        self.message = msg
        self.message = str(self.message.decode("utf-8"))
        self.callback_time = utime.ticks_ms()
        if self.message == "stop":
            self.is_waiting = True
            self.time_to_work = None
            _thread.start_new_thread(self.last_status_lights, ())
        else:
            self.time_to_work = int(self.message) * 1000

    def wheel_power_callback(self, msg):
        """recibe la potencia para las llantas y comunica al driver

        Args:
            msg (arlo_control_msgs.WheelsPower): contiene dos campos de tipo float32
        """
        self.left_power = msg.left_power
        self.right_power = msg.right_power
        self.left_power = int(self.left_power)
        self.right_power = int(self.right_power)

        if (
            self.last_left_power != self.left_power
            or self.last_right_power != self.right_power
        ):
            self.arlobot.go(self.left_power, self.right_power)
            self.last_left_power = self.left_power
            self.last_right_power = self.right_power

    def presence_callback(self, msg):
        """turns off uvc depending on people presence

        Args:
            msg (std_msgs.Bool): it is just a boolean
        """
        msg = msg.data
        if msg:
            self.is_waiting = True
            self.time_to_work = None
            _thread.start_new_thread(self.last_status_lights, ())

    def battery_voltage_callback(self, timer):
        """reads voltage level on battery

        Args:
            timer (managed by modules timer): managed by modules timer
        """
        voltaje_lectura = self.voltage_read.read()
        self.connect_sub = utime.ticks_ms()
        self.battery_voltage_msg.data = float(voltaje_lectura)
        # print(self.battery_voltage_msg.data)
        self.node.publish(self.battery_topic, self.battery_voltage_msg)

    def mqtt_handler(self):
        """handles mqtt client connection"""
        while True:
            try:
                if not self.client.is_conn_issue():
                    self.client.check_msg()
                    self.client.send_queue()
                else:
                    self.client.reconnect()
                    self.client.resubscribe()
            except ConnectionError:
                pass

    def start(self):
        """manages threads starting"""
        self.arlobot.clear_counts()
        self.node.subscribe(
            self.wheel_power_topic, WheelsPower, self.wheel_power_callback
        )
        self.node.subscribe(self.presence_stop_topic, Bool, self.presence_callback)
        while True:
            self.pir_value_msg.data = self.pir_pin.value()
            self.node.publish(self.pir_value_topic, self.pir_value_msg)
            if self.time_to_work is not None:
                try:
                    if utime.ticks_ms() - self.callback_time > self.time_to_work:
                        self.is_waiting = True
                        self.time_to_work = None
                        _thread.start_new_thread(self.last_status_lights, ())
                    else:
                        self.light_controller_active()
                except RuntimeError:
                    pass
            else:
                self.initial_status_lights()
            try:
                wheel_vel = self.arlobot.read_speeds()
                left = int(wheel_vel[0])
                right = int(wheel_vel[1])
                if isinstance(left, int) and isinstance(right, int):
                    self.wheels_velocity.left_encoder = left
                    self.wheels_velocity.right_encoder = right
                    self.node.publish(
                        self.wheel_velocity_topic, self.wheels_velocity
                    )  # wheel speed value published
            except ValueError:
                pass


my_arlobot = ArlorobotMove()
my_arlobot.start()
