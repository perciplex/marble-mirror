import logging
from time import sleep
import os
from typing import Protocol

# TODO: Find an appropriate env to know if we are on the PI.
if os.getenv("PI") is True:
    from adafruit_servokit import ServoKit


class Gate(Protocol):
    chanel: int
    open_angle: float
    closed_angle: float

    def drop(self, delay=0.2) -> None:
        raise NotImplementedError


class SimGate(Gate):
    def __init__(self, channel=0, open_angle=0, closed_angle=180) -> None:
        pass

    def drop(self, delay=0.2) -> None:
        pass


class PiGate(Gate):
    def __init__(self, channel=0, open_angle=0, closed_angle=180) -> None:
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

    def drop(self, delay=0.2) -> None:
        """Open the gate, sleep the delay, then close the gate, and sleep the delay.
        Default is delay=0.2ms"
        """
        self.open()
        sleep(delay)
        self.close()
        sleep(delay)
