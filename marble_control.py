import logging
from enum import Enum
from time import sleep

from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
from adafruit_motor import stepper, servo
import board
from adafruit_tcs34725 import TCS34725
import RPi.GPIO as GPIO

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

class LimitSwitch():
    def __init__(self, pin=21):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    @property
    def is_down(self):
        return GPIO.input(self.pin)

class Pixel:
    BallState = Enum('BallState', 'Black White Empty')
    thresholds = [
        (BallState.Black, (200, 400)),
        (BallState.White, (500, 800)),
        (BallState.Empty, (900, 1000))
    ]
    def __init__(self):
        self.pixel = TCS34725(board.I2C())

    @property
    def value(self):
        lux = self.pixel.lux
        for (ball_state, (low, high)) in self.thresholds:
            if low < lux < high:
                return ball_state
        raise ValueError("Measurement out of range.")