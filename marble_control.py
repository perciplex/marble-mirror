import logging
from time import sleep

from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
from adafruit_motor import stepper, servo


class Gate:
    def __init__(self, channel=0, open_angle=0, closed_angle=180):
        self.servo_kit = ServoKit(channels=16)
        self.servo = self.servo_kit.servo[channel]
        self.open_angle = open_angle
        self.closed_angle = closed_angle

    def open(self):
        self.servo.angle = self.open_angle

    def close(self):
        self.servo.angle = self.closed_angle

    def drop(self, delay=1):
        self.open()
        sleep(delay)
        self.close()
        sleep(delay)


class Stepper:
    def __init__(self, channel=1):
        self.kit = MotorKit()
        self.stepper = getattr(self.kit, f"stepper{channel}")

    def move(self, steps, direction=1):
        if steps < 0:
            steps = -steps
            direction = -1

        if direction == 1:
            direction = stepper.FORWARD
        elif direction == -1:
            direction = stepper.BACKWARD

        for i in range(steps):
            self.stepper.onestep(direction=direction, style=stepper.DOUBLE)

    def __del__(self):
        self.stepper.release()


"""
class MarbleMirror():
    columnoffset = 100
    col_steps = 10
    def __init__(self):
        self.step = 0
        self.kit = MotorKit(i2c=board.I2C())
    def goto_col(self, col):
        goalstep = columnoffset + col*col_steps
        dstep = self.step - goalstep
        if dstep > 0:
            for i in range(dstep):
                self.step += 1
                self.kit.stepper1.onestep(direction=stepper.FORWARD)
        if dstep < 0:
            for i in range(abs(dstep)):
                self.step -= 1
                self.kit.stepper1.onestep(direction=stepper.BACKWARD)
    def home(self):
        while self.step > 0:
            self.step -= 1
            self.kit.stepper1.onestep(direction=stepper.BACKWARD)
    def drop(self):
        self.kit.
"""
