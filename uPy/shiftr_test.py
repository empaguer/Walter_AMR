"""Este es un test para hacer una prueba sencilla de publish con shiftr
"""

import paho.mqtt.client as mqtt

# conexion a shiftr
# mqtt credentials for activation
SHIFTR_TOKEN = "v4BmtoHRLVPHcfc4"  # este token lo da shift
CLIENT_ID = "arlobot_uv_controller"  # el client id puede ser cualquiera
# pero no se debe de repetir con otros nodos
_USUARIO = "paintbug935"  # esto si lo da shiftr
TOPIC_UV_ACTIVACION = "tiempoActivacion"  # y de ahi todos los topic a los
# que se quiere publicar o subscribir
TOPIC_CARGADOR = "accionarCargador"


shiftr_mqtt = mqtt.Client(CLIENT_ID)
shiftr_mqtt.username_pw_set(_USUARIO, password=SHIFTR_TOKEN)
shiftr_mqtt.connect("paintbug935.cloud.shiftr.io", port=1883)
shiftr_mqtt.publish(TOPIC_CARGADOR, str(1))
