"""Esta es una prueba de MQTT con shiftr
"""
from time import sleep

from umqtt.robust import MQTTClient
import network


wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("NETLIFE-Silva", "SASM3141")
sleep(2)


def _cb(topic, msg):
    print(msg)


SHIFTR_TOKEN = b"v4BmtoHRLVPHcfc4"
CLIENT_ID = b"esp3222"
USUARIO = b"paintbug935"
TOPIC = b"tiempoActivacion"  # el topic define a que device en especifico es que se va a subir datos
# b"/v1.6/devices/{NOMBRE_DISPOSITIVO}" en el que NOMBRE_DISPOSITIVO es quien
# define entre los devices creados al cual se quiere subir el dato

client = MQTTClient(
    CLIENT_ID,
    b"paintbug935.cloud.shiftr.io",
    1883,
    user=USUARIO,
    password=SHIFTR_TOKEN,
)  # creacion de objeto
client.connect()  # conexion a ubidots
client.set_callback(_cb)
client.subscribe(TOPIC)

while True:
    try:
        client.wait_msg()

    except ConnectionError as e:
        print(e)
        client.disconnect()
