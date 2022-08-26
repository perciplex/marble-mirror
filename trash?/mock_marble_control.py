import logging
from enum import Enum
import random
from time import sleep
from adafruit_motor import stepper


class ServoKit:
    servo = {}

    def __init__(self, channels):
        for _ in range(channels):
            self.servo[_] = Servo(_)


class Servo:
    def __init__(self, id):
        pass


class AdaStepperMotor:
    def release(self):
        pass

    def onestep(self, direction, style):
        pass


class MotorKit:
    stepper1 = AdaStepperMotor()
    stepper2 = AdaStepperMotor()


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
        logging.debug(f"Dropping servo {self.servo}")
        self.open()
        logging.debug(f"Sleeping for {delay} on servo {self.servo}")
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
        """Take N steps in a specific direction while the stpper.can_step().
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

    @property
    def is_pressed(self):
        return True


class BallState(Enum):
    Black = 1
    White = 0
    Empty = -1
    Unknown = -2


BallColor = {BallState.Black: "⚫", BallState.White: "⚪", BallState.Empty: "🈳"}


class BallReader:
    vocab = [BallState.Empty, BallState.Black, BallState.White]

    def __init__(self, model_pickle_path="model_garbus.pickle"):
        pass

    @property
    def color(self):
        logging.info("Reading current ball color")
        color = self.vocab[random.randrange(0, len(self.vocab))]

        raw = None

        logging.debug(f"Ball read color result: {color} (px value {raw})")
        logging.info(f"Detected ball color {color} {BallColor[color]}")
        return color
