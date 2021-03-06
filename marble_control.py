import logging
from enum import Enum
from time import sleep
from typing import List, Optional, Any

from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
from adafruit_motor import stepper, servo
import board
from adafruit_tcs34725 import TCS34725
import RPi.GPIO as GPIO

import pickle


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


class StepperMotor:
    def __init__(self, channel: int) -> None:
        self.kit = MotorKit()
        self.stepper = getattr(self.kit, f"stepper{channel}")

    def take_step(self, direction: int, style: Any) -> bool:
        self.stepper.onestep(direction=direction, style=style)
        return True

    def move(self, steps: int, direction: int) -> None:
        assert steps >= 0, 'Do not pass negative steps right now'
        if steps < 0:
            steps = -steps
            direction = -1

        assert direction == stepper.FORWARD or direction == stepper.BACKWARD

        '''
        if direction == 1:
            direction = stepper.FORWARD
        elif direction == -1:
            direction = stepper.BACKWARD
        else:
            raise
        '''

        for _ in range(steps):
            step_success = self.take_step(direction=direction, style=stepper.DOUBLE)
            if not step_success:
                break
    
    def release(self):
        self.stepper.release()

    def __del__(self):
        self.release()


class LimitSwitch:
    def __init__(self, pin=21):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    @property
    def is_pressed(self):
        return GPIO.input(self.pin)


class BallState(Enum):
    Black = 1
    White = 0
    Empty = -1
    Unknown = -2


class BallReaderKNN:
    vocab = [BallState.Empty, BallState.Black, BallState.White]
    def __init__(self, model_pickle_path='model_garbus.pickle'):
        self.pixel = TCS34725(board.I2C())
        self.pixel.integration_time = 2.4
        self.pixel.gain = 4

        with open(model_pickle_path, 'rb') as handle:
            self.model = pickle.load(handle)

    @property
    def color(self):
        raw = None
        while raw is None or 0 in raw:
            print("raw", raw)
            raw = self.pixel.color_raw
        logging.error(f"Pixel value {raw}")
        label = self.model.predict([raw])
        return self.vocab[label[0]]


class BallReader:
    thresholds = [
        (BallState.Black, (300, 500)),  # 700 nominal
        (BallState.White, (800, 1500)),  # 1700 nominal
        (BallState.Empty, (550, 650)),  # 1200 nominal
        (BallState.Unknown, (0, 10000)),  # 1200 nominal
    ]

    def __init__(self):
        self.pixel = TCS34725(board.I2C())
        self.pixel.integration_time = 2.4
        self.pixel.gain = 4

    @property
    def color(self):
        lux = self.pixel.lux

        return BallState.White
        for (ball_state, (low, high)) in self.thresholds:
            if low < lux < high:
                return ball_state
        raise ValueError(f"Measurement {lux} out of range.")

    def all_color_info(self):
        lux = self.pixel.lux
        color_rgb_bytes = str(self.pixel.color_rgb_bytes).replace(' ', '')
        color_temperature = str(self.pixel.color_temperature).replace(' ', '')
        color_raw = str(self.pixel.color_raw).replace(' ', '')
        all_ball_info = f'{color_rgb_bytes}\t{color_raw}\t{color_temperature}\t{lux}\n'
        return self.pixel.color_raw