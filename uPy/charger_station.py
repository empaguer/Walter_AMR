"""This script is for the esp32 in the charging station
"""

import gc
import utime
from machine import Pin
import network
from umqtt.robust2 import MQTTClient

gc.enable()


def do_connect():
    """Helps to connect to wifi"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect("SSID", "PASSWORD")
        while not wlan.isconnected():
            pass
    # print("network config:", wlan.ifconfig())


do_connect()

utime.sleep(20)

SHIFTR_TOKEN = b"v4BmtoHRLVPHcfc4"
CLIENT_ID = b"ESP32_charger"
USUARIO = b"paintbug935"
TOPIC_TIME = b"tiempoActivacion"
TOPIC_STOP = b"stop"
TOPIC_CARGADOR = b"accionarCargador"

client = MQTTClient(
    CLIENT_ID,
    b"paintbug935.cloud.shiftr.io",
    1883,
    user=USUARIO,
    password=SHIFTR_TOKEN,
)  # creacion de objeto

client.DEBUG = True
# Information whether we store unsent messages with the flag QoS==0 in the queue.
client.KEEP_QOS0 = False
client.NO_QUEUE_DUPS = True
# Limit the number of unsent messages in the queue.
client.MSG_QUEUE_MAX = 1

relay_pin = Pin(22, Pin.OUT)


def charger_activate_callback(topic, msg, retained, duplicate):
    """manages relay activation depending on the incoming msg value

    Args:
        topic (managed by module): managed by module
        msg (managed by module): managed by module
        retained (managed by module): managed by module
        duplicate (managed by module): managed by module
    """
    message = msg
    message = str(message.decode("utf-8"))
    if message == "1":
        relay_pin.on()
    else:
        relay_pin.off()


client.set_callback(charger_activate_callback)
client.connect(clean_session=False)  # conexion a broker
client.subscribe(TOPIC_CARGADOR)

while True:
    while True:
        try:
            if not client.is_conn_issue():
                client.check_msg()
                client.send_queue()
            else:
                client.reconnect()
                client.resubscribe()
        except ConnectionError:
            pass
