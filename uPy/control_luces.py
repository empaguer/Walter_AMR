"""this scripts tests light control
"""

import time
import _thread
from umqtt.robust import MQTTClient
import network

from machine import Pin, ADC
import machine


# Definition of variables
# Pins
l_uvc = Pin(26, Pin.OUT)
l_red = Pin(33, Pin.OUT)
l_green = Pin(25, Pin.OUT)
l_blue = Pin(32, Pin.OUT)
message = ""
active = False
time_to_work = 0
time_working = 0


voltage_read = ADC(Pin(34))
voltage_read.atten(ADC.ATTN_11DB)


def battery_voltage_callback(timer):
    """timed function to meassure battery voltage

    Args:
        timer (managed by timer module): [descrimanaged by timer module
    """
    voltaje_lectura = voltage_read.read()
    voltaje_lectura = float(0.0043716 * voltaje_lectura)
    # print(self.battery_voltage_msg.data)
    client.publish(b"bateria_voltaje", bytes(str(voltaje_lectura), "utf-8"))


timer = machine.Timer(0)
timer.init(
    period=6000,
    mode=machine.Timer.PERIODIC,
    callback=battery_voltage_callback,
)

##Functions
def do_connect():
    """connects esp32 to wifi"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("connecting to network...")
        wlan.connect("NETLIFE-Silva", "SASM3141")
        while not wlan.isconnected():
            pass
    print("network config:", wlan.ifconfig())


def _cb(topic, msg):
    """callback to store incoming message

    Args:
        topic (managed by module): [descrimanaged by module
        msg (incoming message): [descriincoming message
    """
    global message
    message = msg


def initial_status_lights():
    l_uvc.value(1)
    l_red.value(1)
    l_green.value(0)
    l_blue.value(1)


def wait_after_desinfection():
    for i in range(2):
        l_uvc.value(1)
        l_red.value(0)
        l_green.value(0)
        l_blue.value(1)
        time.sleep(1)
    initial_status_lights()


def wait_before_desinfection():
    for i in range(2):
        l_uvc.value(1)
        l_red.value(0)
        l_green.value(0)
        l_blue.value(1)
        time.sleep(1)


def msg_overlooker():

    global active
    global time_to_work
    global time_working
    while True:
        try:
            client.wait_msg()
            translated_msg = str(message.decode("utf-8"))

            if translated_msg == "stop":
                time_working = time_to_work
            else:
                wait_before_desinfection()
                time_to_work = int(translated_msg)
                if time_to_work > 0:
                    active = True

        except ConnectionError:
            pass


def ligth_controller():
    """controls light usage"""
    global active
    global time_to_work
    global time_working
    time_working = 0
    while True:
        if active:

            time_working += 1
            l_uvc.value(0)
            l_red.value(0)
            l_green.value(1)
            l_blue.value(1)
            print("time working", time_working)
            if time_working >= time_to_work:
                time_working = 0
                active = False
                wait_after_desinfection()

        time.sleep(1)


##
initial_status_lights()
do_connect()

SHIFTR_TOKEN = b"v4BmtoHRLVPHcfc4"
CLIENT_ID = b"ESP32"
USUARIO_ = b"paintbug935"
TOPIC_TIME = b"tiempoActivacion"
TOPIC_STOP = b"stop"

client = MQTTClient(
    CLIENT_ID,
    b"paintbug935.cloud.shiftr.io",
    1883,
    user=USUARIO_,
    password=SHIFTR_TOKEN,
)  # creacion de objeto
client.connect()  # conexion a ubidots
client.set_callback(_cb)
client.subscribe(TOPIC_TIME)
client.subscribe(TOPIC_STOP)


_thread.start_new_thread(msg_overlooker, ())
_thread.start_new_thread(ligth_controller, ())
