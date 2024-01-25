# pins and serial library
from machine import UART
from utime import sleep, sleep_ms, ticks_ms

# object to control de DHB-10 driver
class ArloRobot(object):

    # com packet sending
    def com(self, packet, ret=False):

        msg = ""
        for i in packet:
            msg = msg + i + " "
        msg = msg[:-1] + "\r"
        if not ret:
            self.uart.write(bytes(msg, "utf-8"))
        else:
            for i in msg:
                self.uart.write(bytes(i, "utf-8"))
                # sleep_ms(self.pace)

        resp = ""
        data = ""
        if ret:
            while str(data) != str(b"\r") and data is not None:
                data = self.uart.read(1)
                # print(data)
                # sleep_ms(self.pace)
                if str(data) != str(b"\r"):
                    try:
                        resp = resp + str(data)[2:][:-1]
                    except Exception as e:
                        print(e)
            return resp
        else:
            self.uart.read()

    # set up/set down
    # serialid is defined as the ID of the serial bus from the
    # microcontroller, however tx and rx can be defined
    def __init__(self, serial_id=2, baudrate=19200, timeout=30, pace=0, **kwargs):
        self.baudrate = baudrate
        self.serial_id = serial_id
        self.pace = 0
        self.timeout = timeout

        if "tx" in kwargs and "rx" in kwargs:
            self.uart = UART(self.serial_id, 19200)
            self.uart.init(
                19200,
                tx=kwargs.get("tx"),
                rx=kwargs.get("rx"),
                bits=8,
                parity=None,
                stop=1,
                txbuf=0,
                timeout=self.timeout,
            )
            print("TX pin and RX Pin defined.")
        else:
            self.uart = UART(self.serial_id, 19200)
            self.uart.init(
                19200, bits=8, parity=None, stop=1, txbuf=0, timeout=self.timeout
            )

        self.com(["TXPIN", "CH2"])
        sleep(0.5)
        self.com(["RXPIN", "CH1"])
        sleep(0.5)
        self.com(["DEC"])
        sleep(0.5)
        self.com(["BAUD", str(self.baudrate)])
        sleep(0.5)

        if "tx" in kwargs and "rx" in kwargs:
            self.uart = UART(self.serial_id, self.baudrate)
            self.uart.init(
                self.baudrate,
                tx=kwargs.get("tx"),
                rx=kwargs.get("rx"),
                bits=8,
                parity=None,
                stop=1,
                txbuf=0,
                timeout=self.timeout,
            )
        else:
            self.uart = UART(self.serial_id, self.baudrate)
            self.uart.init(
                self.baudrate,
                bits=8,
                parity=None,
                stop=1,
                txbuf=0,
                timeout=self.timeout,
            )

        self.com(["PACE", str(pace)])
        self.pace = pace
        sleep(0.5)

    # end serial connection
    def end(self):
        self.uart.deinit()

    # -------------------------- movements methods------------------------

    # Turn command
    # motor_movements corresponds to the amount of encode positions
    # top_speed to the positions per second
    def turn(self, motor_movement, top_speed, ret=False):
        self.com(["TURN", str(motor_movement), str(top_speed)], ret)

    # arc turns the motors so that the platform moves along the arc of a circle
    # of a given radius with a speed and an angle
    def arc(self, radius, top_speed, angle, ret=False):
        self.com(["ARC", str(radius), str(top_speed), str(angle)], ret)

    # left/right -> -32767 to 32767
    # speed -> 1 to 32767
    def move(self, left, right, speed, ret=False):
        self.com(["MOVE", str(left), str(right), str(speed)], ret)

    # left/right -> -32767 to 32767
    def go_speed(self, left, right, ret=False):
        self.com(["GOSPD", str(left), str(right)], ret)

    # left/right -> -127 to 127
    def go(self, left, right, ret=False):
        self.com(["GO", str(left), str(right)], ret)

    def travel(self, distance, top_speed, angle, ret=False):
        self.com(["TRVL", str(distance), str(top_speed), str(angle)], ret)

    # --------------------------- information methods -----------------------

    def read_counts(self, ret=True):
        counts = self.com(["DIST"], ret)
        try:
            counts = counts.split(" ")
        except Exception as e:
            print(e)
        return counts

    def read_left_counts(self, ret=True):
        return self.com(["DIST"], ret)[0]

    def read_right_counts(self, ret=True):
        return self.com(["DIST"], ret)[1]

    def read_speeds(self, ret=True):
        speeds = self.com(["SPD"], ret)
        try:
            speeds = speeds.split(" ")
        except Exception as e:
            print(e)
        return speeds

    def read_left_speed(self, ret=True):
        return self.com(["SPD"], ret)[0]

    def read_right_speed(self, ret=True):
        return self.com(["SPD"], ret)[1]

    def read_head_angle(self, ret=True):
        data = self.com(["HEAD"], ret)
        try:
            if len(data) == 2:
                return int((data[1]))
            else:
                return int(data[0])
        except:
            return data

    def read_firmware_ver(self, ret=True):
        return self.com(["VER"], ret)

    def read_hardware_ver(self, ret=True):
        return self.com(["HWVER"], ret)

    def clear_counts(self, ret=True):
        return self.com(["RST"], ret)

    # ---------------------------- communication modes -----------------------

    def write_pulse_mode(self, ret=False):
        return self.com(["PULSE"], ret)

    def set_lf_mode(self, status, ret=False):
        return self.com(["SETLF", str(status)], ret)

    def set_hex_com(self, ret=False):
        return self.com(["HEX"], ret)

    def set_dec_com(self, ret=False):
        return self.com(["DEC"], ret)

    def set_echo_mode(self, status, ret=False):
        return self.com(["ECHO", str(status)], ret)

    def set_verbose_mode(self, status, ret=False):
        return self.com(["VERB", str(status)], ret)

    def set_rx_pin(self, pin, ret=False):
        return self.com(["RXPIN", str(pin)], ret)

    def set_tx_pin(self, pin, ret=False):
        return self.com(["TXPIN", str(pin)], ret)

    def set_baud_rate(self, baud, ret=False):
        return self.com(["BAUD", str(baud)], ret)

    def set_pwm_scale(self, scale, ret=False):
        return self.com(["SCALE", str(scale)], ret)

    def set_pace(self, pace, ret=False):
        return self.com(["PACE", str(pace)], ret)

    def set_hold(self, hold, ret=False):
        return self.com(["HOLD", str(baud)], ret)

    # -------------------------- closed loop constants ----------------------

    def set_ki_limit(self, limit, ret=False):
        return self.com(["KIP", str(limit)], ret)

    def set_ki_decay(self, decay, ret=False):
        return self.com(["KIT", str(decay)], ret)

    def set_kimax(self, maxim, ret=False):
        return self.com(["KIMAX", str(maxim)], ret)

    def set_ki_constant(self, constant, ret=False):
        return self.com(["KI", str(constant)], ret)

    def set_kp_constant(self, constant, ret=False):
        return self.com(["KP", str(constant)], ret)

    def set_acc_rate(self, acc, ret=False):
        return self.com(["ACC", str(acc)], ret)

    def set_ramp_rate(self, rate, ret=False):
        return self.com(["RAMP", str(rate)], ret)

    def set_live_zone(self, limit, ret=False):
        return self.com(["LZ", str(limit)], ret)

    def set_dead_zone(self, limit, ret=False):
        return self.com(["DZ", str(limit)], ret)

    def set_ppr(self, ppr, ret=False):
        return self.com(["PPR", str(ppr)], ret)

    # -------- config ----------
    def restore_config(self, ret=False):
        return self.com(["RESTORE"], ret)

    def read_config(self, command, ret=True):
        return self.com([command], ret)


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))
