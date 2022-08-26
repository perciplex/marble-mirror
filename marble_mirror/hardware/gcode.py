import serial
import re
from time import sleep
from typing import Protocol

# We somehow set up Lim to be on by changing some flags.
# This gets saved in EEPROM


class GCodeBoard(Protocol):
    port: str
    baudrate: int
    home: bool

    def move(self, axis, pos):
        raise NotImplementedError

    def move_Y_n_rotation(self, n):
        raise NotImplementedError


class SimGCodeBoard(GCodeBoard):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, home=True):
        pass

    def move(self, axis, pos):
        pass

    def move_Y_n_rotation(self, n):
        pass


class PiGCodeBoard(GCodeBoard):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, home=True):
        # setup that good feed rate
        self.serial = serial.Serial(port, baudrate=baudrate)  # open serial port 115200

        self.serial.reset_input_buffer()
        # Clear the first two erroneous messages
        self.serial.readline()
        self.serial.readline()

        # Set directions X neg, Y neg
        self.write(f"$3=3")

        # Homing cycle enabled
        self.write(f"$22=1")
        # Invert home direction for x, y, and z
        self.write(f"$23=7")
        # Set home step-off in mm
        self.write(f"$27=3")
        # Set X step/mm to (200 steps / rev) * (16 microsteps / ste) / (8 mm / rev) = 400 microsteps/mm
        self.write(f"$100={400}")

        # Set Y step/mm to (200 steps / rev) * (16 microsteps / ste) = 3200 microsteps/rev
        self.write(f"$101={200*16}")

        # Set max X speed to 4000 mm/min, accel to 200 mm/ss
        self.write("$110=6000")  # 5000 working
        self.write("$120=1000")  # 5000 working

        # Set max Y speed to 8000 mm/min, accel to 2000 mm/ss
        self.write("$111=25")
        if home:
            self.home()

        self.serial.reset_input_buffer()

    def write(self, msg):
        self.serial.write(f"{msg}\n".encode())
        sleep(0.1)

    def clear_alarm(self):
        self.write(f"$X")

    def move(self, axis, pos):
        self.write(f"G0 {axis}{pos}")
        self.block_until_move()

    def move_Y_n_rotation(self, n):
        self.write("G91")
        self.write(f"Y{n}")
        self.write("G90")
        self.block_until_move()

    def block_until_move(self):
        while "Idle" not in str(self.status_string()):
            sleep(0.01)
            # print('waiting for idle...')

    def home(self):
        # Only home the X
        # https://github.com/gnea/grbl/blob/bfb67f0c7963fe3ce4aaf8a97f9009ea5a8db36e/grbl/config.h#L120
        self.write("$HX")

    def status_string(self):
        self.serial.reset_output_buffer()
        self.serial.reset_input_buffer()
        self.write("?")
        status = self.serial.readline()
        # print(status)
        return status

    def get_settings(self):
        self.serial.reset_output_buffer()
        self.serial.reset_input_buffer()
        self.write("$$")
        last = ""
        while "ok" not in str(last):
            last = self.serial.readline()
            print(last)

    def lim(self):
        status = self.status_string()
        # WARNING! X and Z are reversed!
        result = re.search(r"Pn:(?P<X>X?)(?P<Y>Y?)(?P<Z>Z?)", str(status))
        return {k: bool(int(v)) for k, v in result.groupdict().items()}

    def pos(self):
        status = self.status_string()
        result = re.search(r"MPos:(?P<X>\d+\.\d+),(?P<Y>\d+\.\d+),(?P<Z>\d+\.\d+)", str(status))
        return {k: float(v) for k, v in result.groupdict().items()}

    def reset_alarm(self):
        # unlock in case of alarm lock
        # https://github.com/gnea/grbl/wiki/Grbl-v1.1-Commands#x---kill-alarm-lock
        self.write("$X")

    # def __del__(self):
    #    self.serial.close()
