#!/usr/bin/env python3

from .arlorobot import *
import time

arlobot = ArloRobot(serial_id="/dev/ttyUSB0", baudrate=19200)
time.sleep(1)
result = arlobot.read_counts()
print(result)
arlobot.set_baud_rate(115200, True)
arlobot.close()
arlobot = ArloRobot(serial_id="/dev/ttyUSB0", baudrate=115200)
time.sleep(1)
arlobot.clear_counts(True)
arlobot.set_echo_mode(0)
arlobot.move(150, 150, 100, True)
time.sleep(4)
result = arlobot.read_counts()
print(result)
arlobot.close()
