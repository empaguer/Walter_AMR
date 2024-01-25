from time import sleep
import serial
from serial.serialutil import Timeout

ser = serial.Serial(
    "/dev/ttyTHS1",
    19200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)  # open serial port
ser.write(b"TXPIN CH2\r")
sleep(1)
ser.write(b"RXPIN CH1\r")
sleep(1)
ser.write(b"BAUD 19200\r")
sleep(1)
ser.write(b"PACE 2\r")
sleep(1)
ser.write(b"DEC\r")
sleep(1)
ser.write(b"ECHO OFF\r")
sleep(1)
ser.write(b"VERB ON\r")
sleep(1)
ser = serial.Serial(
    "/dev/ttyTHS1",
    19200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
sleep(1)
ser.write(b"RST\r")
sleep(1)
ser.write(b"GOSPD 0 0\r")
sleep(1)
print("enviando valores")
val = 0.002
while True:
    ser.write(b"D")
    sleep(val)
    ser.write(b"I")
    sleep(val)
    ser.write(b"S")
    sleep(val)
    ser.write(b"T")
    sleep(val)
    ser.write(b"\r")
    sleep(val)
    resp = ""
    data = ""
    print("pidiendo dist")
    while data != b"\r" and data is not None:
        data = ser.read(1)
        sleep(val)
        print(data)
        resp = resp + str(data)[2:][:-1]
    print(resp[:-2])
