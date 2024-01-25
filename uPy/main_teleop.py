"""Este es un test para hacer una teleoperacion manual desde python
"""

import usocket as socket
import network  # web connecting
import utime
import _thread
import uros
from arlorobot import ArloRobot


from std_msgs import Int16

SSID = "WIFI_SSID"  # wifi credentials
PASS = "PASSWORD"

wifi = network.WLAN(network.STA_IF)  # connecting to wifi
wifi.active(True)
wifi.connect(SSID, PASS)

utime.sleep(2)


class ArlorobotMove:
    """Esta clase nos permite lanzar dos threads
    para mover la plataforma y publicar tambien los datos de encoders
    """

    def __init__(self):
        self.arlobot = ArloRobot(serial=1, tx=12, rx=13, baudrate=115200)
        self.node = uros.NodeHandle(0, 115200)
        self.left_encoder = Int16()
        self.right_encoder = Int16()
        self.left_encoder_topic = "/lwheel"
        self.right_encoder_topic = "/rwheel"
        self.linear_speed = 10
        self.angular_speed = 10

    def move_thread(self):
        """function to manage platform movements"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(("", 88))

        sock.listen(10)
        print("escuchando")
        while True:
            connection, client_address = sock.accept()
            try:
                while True:

                    data = connection.recv(4096)
                    if data != b"":
                        data = int(data.decode("utf-8"))
                        print(data)

                    if data == 0:  # adelante
                        self.arlobot.go_speed(self.linear_speed, self.linear_speed)
                    elif data == 1:  # atras
                        self.arlobot.go_speed(-self.linear_speed, -self.linear_speed)
                    elif data == 2:  # izquierda
                        self.arlobot.go_speed(-self.linear_speed, self.linear_speed)
                    elif data == 3:  # derecha
                        self.arlobot.go_speed(self.linear_speed, -self.linear_speed)

            finally:
                connection.close()

    def encoders_thread(self):
        """permite hacer la publicacion del valor de los encoders"""
        while True:
            left = self.arlobot.read_left_counts()  # encoders read from arlorobot
            right = self.arlobot.read_right_counts()
            self.left_encoder.data = left  # messages defined
            self.right_encoder.data = right
            self.node.publish(
                self.left_encoder_topic, self.left_encoder
            )  # encoders value published
            self.node.publish(self.right_encoder_topic, self.right_encoder)
            utime.sleep(1)

    def start_threads(self):
        """func to start both threads"""
        _thread.start_new_thread(self.move_thread, ())
        _thread.start_new_thread(self.encoders_thread, ())


if __name__ == "__main__":
    my_arlobot = ArlorobotMove()
    my_arlobot.start_threads()
