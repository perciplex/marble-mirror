import logging
import pickle
from enum import Enum
from time import sleep
import numpy as np
import os
import random
from typing import Protocol

# TODO: Find an appropriate env to know if we are on the PI.
if os.getenv("PI") is True:
    import picamera


class BallState(Enum):
    Black = 1
    White = 0
    Empty = -1


BallColor = {BallState.Black: "⚫", BallState.White: "⚪", BallState.Empty: "🈳"}


class Camera(Protocol):
    @property
    def color(self) -> BallState:
        raise NotImplementedError

    @property
    def color_raw(self):
        raise NotImplementedError


class SimCamera(Camera):
    def __init__(self, model_pickle_path=None):
        self.rng = random.Random(1337)

    @property
    def color(self) -> BallState:
        color = self.rng.choice(list(BallState))
        logging.info(f"Detected ball color {color} {BallColor[color]}")
        return color


class PiCamera(Camera):
    color_str_to_enum = {
        "empty": BallState.Empty,
        "black": BallState.Black,
        "white": BallState.White,
    }

    def __init__(self, model_pickle_path=None):
        self._camera = picamera.PiCamera()
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
