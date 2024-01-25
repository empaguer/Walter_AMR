from time import sleep, time
import serial
from serial.serialutil import Timeout

ser = serial.Serial("/dev/ttyTHS1", 115200)

while True:
    data = ser.readline()
    print(data)

    # ser.write(b"hola\n\r")
