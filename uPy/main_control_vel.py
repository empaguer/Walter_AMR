"""permite todas las funcionalidades del hardware del robot de motores
y control de lamparas
"""

import gc
import _thread
import uros
import utime
import network
import machine
from machine import Pin, ADC
from umqtt.robust2 import MQTTClient
from arlo_control_msgs import WheelsEncoders  # rosserial messages
from arlorobot import ArloRobot
from std_msgs import Float32, Bool


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
    """Manages all functionality of the robot with the esp32"""

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
        self.wheel_velocity_topic = "/wheels_vel"
        self.battery_topic = "/battery_voltage"
        self.presence_stop_topic = "/presence"
        self.switch_value_topic = "/dock_switch_value"

        self.arlobot = ArloRobot(serial_id=2, baudrate=19200, tx=17, rx=16, pace=2)
        self.node = uros.NodeHandle(1, 115200, tx=1, rx=3)
        # , tx=1, rx=3
        self.left_power = 0
        self.right_power = 0

        self.last_left_power = 0
        self.last_right_power = 0

        self.battery_voltage_msg = Float32()

        self.dock_switch_msg = Bool()

        self.switch_pin = machine.Pin(35, machine.Pin.IN)

        _thread.start_new_thread(self.mqtt_handler, ())

    # funciones de control de luces
    def initial_status_lights(self):
        """setup for rgb lights"""
        if not self.is_waiting:
            self.l_uvc.value(1)
            self.l_red.value(1)
            self.l_green.value(0)
            self.l_blue.value(1)

    def last_status_lights(self):
        """sets last value of lights"""
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
        """sets lights status as active uv"""
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

    def wheel_velocity_callback(self, msg):
        """recibe el valor de velocidades para las llantas y las envia al driver

        Args:
            msg (arlo_control_msgs.WheelsEncoders): used to send wheels velocities of value int16
        """
        self.left_power = msg.left_encoder
        self.right_power = msg.right_encoder
        self.left_power = int(self.left_power)
        self.right_power = int(self.right_power)

        if (
            self.last_left_power != self.left_power
            or self.last_right_power != self.right_power
        ):
            self.last_left_power = self.left_power
            self.last_right_power = self.right_power

    def presence_callback(self, msg):
        """If there is people presence, stops uv lights

        Args:
            msg (std_msgs.Bool): ros standard message, send bool
        """
        msg = msg.data
        if msg:
            self.is_waiting = True
            self.time_to_work = None
            _thread.start_new_thread(self.last_status_lights, ())

    def battery_voltage_callback(self, timer):
        """function to measure battery voltage using a timer

        Args:
            timer (managed by module): managed by module
        """
        voltaje_lectura = self.voltage_read.read()
        self.battery_voltage_msg.data = float(0.00412 * voltaje_lectura)
        # print(self.battery_voltage_msg.data)
        self.node.publish(self.battery_topic, self.battery_voltage_msg)

    def mqtt_handler(self):
        """function to reconnect mqtt client"""
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
        """function to start all robot functions"""
        self.arlobot.clear_counts()
        self.node.subscribe(
            self.wheel_velocity_topic, WheelsEncoders, self.wheel_velocity_callback
        )
        self.node.subscribe(self.presence_stop_topic, Bool, self.presence_callback)
        while True:
            self.dock_switch_msg.data = self.switch_pin.value()
            self.node.publish(self.switch_value_topic, self.dock_switch_msg)
            self.arlobot.go_speed(self.left_power, self.right_power)
            utime.sleep_ms(100)
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


my_arlobot = ArlorobotMove()
my_arlobot.start()
