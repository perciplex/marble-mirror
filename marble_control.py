import abc
import logging
import pickle
from enum import Enum
from time import sleep

import board
import RPi.GPIO as GPIO
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
from adafruit_tcs34725 import TCS34725


class Gate:
    def __init__(self, channel=0, open_angle=0, closed_angle=180):
        self.servo_kit = ServoKit(channels=16)
        self.servo = self.servo_kit.servo[channel]
        self.open_angle = open_angle
        self.closed_angle = closed_angle

    def open(self):
        logging.debug(f"Openning servo {self.servo}...")
        self.servo.angle = self.open_angle
        logging.debug(f"Openned servo {self.servo}")

    def close(self):
        logging.debug(f"Closing servo {self.servo}...")
        self.servo.angle = self.closed_angle
        logging.debug(f"Closed servo {self.servo}")

    def drop(self, delay=1):
        self.open()
        sleep(delay)
        self.close()


class StepperMotor:
    def __init__(self, channel: int, step_sleep: int = 0) -> None:
        self.kit = MotorKit()
        self.stepper = getattr(self.kit, f"stepper{channel}")
        self.step_sleep = step_sleep

    def can_step(self, direction):
        """Returns if the motor can step.

        Returns:
            Bool: Bool if the motor is allowed to step.
        """
        return True

    def move(self, steps: int, direction: int) -> None:
        """Take N steps in a specific direction while the stepper.can_step().
        Releases the motor after the move is completed.

        Args:
            steps (int): Number of steps to take. Must be positive int.
            direction (int): Must be either 1 (forward) or 2 (backwards)
        """
        assert steps >= 0, "Do not pass negative steps"
        assert direction in [stepper.FORWARD, stepper.BACKWARD]

        for _ in range(steps):
            if not self.can_step(direction):
                break
            else:
                self.stepper.onestep(direction=direction, style=stepper.DOUBLE)
                sleep(self.step_sleep)
        self.release()

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


BallColor = {BallState.Black: "⚫", BallState.White: "⚪", BallState.Empty: "🈳"}


class BallReader:
    vocab = [BallState.Empty, BallState.Black, BallState.White]

    def __init__(self, model_pickle_path="model_garbus.pickle"):
        self.pixel = TCS34725(board.I2C())
        self.pixel.integration_time = 2.4
        self.pixel.gain = 4

        with open(model_pickle_path, "rb") as handle:
            self.model = pickle.load(handle)

    @property
    def color(self):
        raw = None
        while raw is None or 0 in raw:
            raw = self.pixel.color_raw
        logging.debug(f"Pixel value {raw}")
        label = self.model.predict([raw])
        color = self.vocab[label[0]]
        logging.info(f"Detected ball color {color} {BallColor[color]}")
        return color

class BallReader2:
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
            logging.debug("raw color value", raw)
            raw = self.pixel.color_raw
        logging.debug(f"Pixel value {raw}")
        label = self.model.predict([raw])
        return self.vocab[label[0]]