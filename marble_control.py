import logging
import pickle
import re
import serial
from enum import Enum
from time import sleep
import numpy as np
import os
import random
from typing import Protocol

if os.getenv("PI") == True:
    from adafruit_servokit import ServoKit
    from adafruit_tcs34725 import TCS34725
    from picamera import PiCamera


class Gate(Protocol):
    def drop(self, delay=0.2) -> None:
        raise NotImplemented


class SimGate(Gate):
    def __init__(self, channel=0, open_angle=0, closed_angle=180):
        pass

    def drop(self, delay=0.2) -> None:
        pass


class PiGate(Gate):
    def __init__(self, channel=0, open_angle=0, closed_angle=180):
        self.servo_kit = ServoKit(channels=16)
        self.servo = self.servo_kit.servo[channel]
        self.open_angle = open_angle
        self.closed_angle = closed_angle

    def open(self) -> None:
        logging.debug(f"Openning servo {self.servo}...")
        self.servo.angle = self.open_angle
        logging.debug(f"Openned servo {self.servo}")

    def close(self) -> None:
        logging.debug(f"Closing servo {self.servo}...")
        self.servo.angle = self.closed_angle
        logging.debug(f"Closed servo {self.servo}")

    def drop(self, delay=0.2):
        """Open the gate, sleep the delay, then close the gate, and sleep the delay.
        Default is delay=0.2ms"
        """
        self.open()
        sleep(delay)
        self.close()
        sleep(delay)


class BallState(Enum):
    Black = 1
    White = 0
    Empty = -1


BallColor = {BallState.Black: "⚫", BallState.White: "⚪", BallState.Empty: "🈳"}


class Camera(Protocol):
    @property
    def color(self):
        raise NotImplemented

    @property
    def color_raw(self):
        raise NotImplemented


class SimCamera(Camera):
    def __init__(self, model_pickle_path=None):
        pass

    @property
    def color(self):
        color = random.choice(list(BallState))
        logging.info(f"Detected ball color {color} {BallColor[color]}")
        return color


class PiCamera(Camera):
    color_str_to_enum = {
        "empty": BallState.Empty,
        "black": BallState.Black,
        "white": BallState.White,
    }

    def __init__(self, model_pickle_path=None):
        self._camera = PiCamera()
        self.model = None
        self.vocab = None
        if model_pickle_path is not None:
            with open(model_pickle_path, "rb") as handle:
                model_and_vocab = pickle.load(handle)

            self.model = model_and_vocab["model"]
            self.vocab = model_and_vocab["vocab"]

    @property
    def color(self) -> BallState:
        assert self.model is not None
        assert self.vocab is not None

        raw = None
        while raw is None:
            raw = self.color_raw
        logging.debug(f"Pixel value {raw}")
        label = self.model.predict([raw])
        color_str = self.vocab[label[0]]
        color = self.color_str_to_enum[color_str]
        logging.info(f"Detected ball color {color} {BallColor[color]}")
        return color

    @property
    def color_raw(self):
        self._camera.resolution = (320, 240)
        self._camera.framerate = 24
        sleep(0.05)
        output = np.empty((240, 320, 3), dtype=np.uint8)
        self._camera.capture(output, "rgb")
        return output.flatten()

    def __del__(self):
        print("Closing camera...")
        self._camera.close()
        print("Closed")
