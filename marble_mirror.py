import logging
import numpy as np
from enum import IntEnum
from time import sleep
from typing import List, Optional

import click
from adafruit_motor import stepper

from marble_control import (
    BallReader,
    BallState,
    Gate,
    LimitSwitch,
    StepperMotor,
    GCodeMotor
)
from gcode import GCodeBoard


STEPS_PER_REV = 200.0
MM_PER_REV = 8.0
INTER_COLUMN_DISTANCE = 11.613  # (mm). Original (non rails) was 13.7
# STEPS_PER_COLUMN = int(INTER_COLUMN_DISTANCE * STEPS_PER_REV / MM_PER_REV)
STEPS_PER_COLUMN = 7.874
APPRECIATE_IMAGE_TIME = 5.0
BOARD_DROP_SLEEP_TIME = 1.0
# The wire with a ziptie on it is for the elevator stepper.
ELEVATOR_STEPPER_CHANNEL = 2
CARRIAGE_STEPPER_CHANNEL = 1
CARRIAGE_SERVO_CHANNEL = 0
BOARD_SERVO_CHANNEL = 12
CARRIAGE_SERVO_OPEN_ANGLE = 0
CARRIAGE_SERVO_CLOSE_ANGLE = 15
BOARD_SERVO_OPEN_ANGLE = 110
BOARD_SERVO_CLOSE_ANGLE = 145
LIMIT_SWITCH_GPIO_PIN = 1
HOME_COLUMN_VALUE = 0.  # This needs to get calibrated to home offset from column 0
ELEVATOR_BALL_PUSH_STEPS = 202  # Set intentionally
ELEVATOR_PUSH_WAIT_TIME_S = 0.5  # Wait after pushing a ball before reading
HOME_TO_FIRST_COLUMN_DISTANCE_MM = 8.5

N_COLS = 15
N_ROWS = 8


class ElevatorMoveDirection(IntEnum):
    BALL_UP = stepper.BACKWARD
    BALL_DOWN = stepper.FORWARD


class CarriageMoveDirection(IntEnum):
    AWAY = stepper.BACKWARD
    HOME = stepper.FORWARD


class MarbleBoard:
    def __init__(self, n_cols: int, n_rows: int):
        self._n_cols = n_cols
        self._n_rows = n_rows
        self._board_state = [[] for i in range(n_cols)]
        self._queues_list = None
        self.image = None

    def set_new_board(self, image: List[List[int]]) -> None:
        """
        Transforms the image from a list of rows into a list of columns
        which are used as queues for which column needs what color ball.
        eg, input of
        [
            [0, 1, 2],
            [a, b, c],
            [i, j, k],
            ...
        ]

        yields:
        [
            [0, a, i],
            [1, b, j],
            [2, c, k],
            ...
        ]
        """
        self._queues_list = []
        self.image = image

        image_rows = len(image)
        image_cols = len(max(image, key=len))
        assert image_cols <= self._n_cols, f"Can only have up to {self._n_cols} columns"
        assert image_rows <= self._n_rows, f"Can only have up to {self._n_cols} columns"

        self._queues_list = np.transpose(np.array(image)).tolist()
        self.print_board_queues()

    def print_board_queues(self):
        logging.debug(
            f"Current state of internal queues: {[_ for _ in self._queues_list]}"
        )
        for row in self._board_state:
            logging.debug(row)

    def get_next_valid_column_for_color(self, ball_color: int) -> Optional[int]:
        logging.debug(f"Looking for valid column for ball color: = {ball_color}")
        # Go through each queue (column), see if the bottom matches ball_color.
        # If true, pop from that queue and return the column.
        for col, queue in enumerate(self._queues_list):
            if len(queue) > 0 and queue[-1] == ball_color.value:
                queue.pop()
                self._board_state[col].append(ball_color.value)
                return col

        # If we didn't find any, return None
        return None

    def done(self) -> bool:
        return all([len(q) == 0 for q in self._queues_list])

class Carriage:
    def __init__(self, gcode_board: GCodeBoard) -> None:
        self._ball_dropper = Gate(
            open_angle=CARRIAGE_SERVO_OPEN_ANGLE,
            closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE,
            channel=CARRIAGE_SERVO_CHANNEL,
        )
        self._gcode_board = gcode_board
        # self._carriage_motor = GCodeMotor(channel=CARRIAGE_STEPPER_CHANNEL)
        self._cur_column = None
        self._ball_dropper.drop()

    def drop_ball_in_column_and_home(self, target_column: int) -> None:
        """Go to target column, drop the ball, and then return home.

        Args:
            target_column (int): Target column to drop ball in.
        """
        logging.debug(f"Dropping ball in column {target_column}")
        self.go_to_column(target_column)
        self._ball_dropper.drop()
        self.go_home()

    def go_to_column(self, target_column: int) -> None:
        """Move the carriage to the target column. Calculates the inter-column
        distance, converts to stepper steps, and determines the direction.

        If the current column is not known, first go home to calibrate.

        Args:
            target_column (int): The target column to go to.
        """

        logging.debug(
            f"Carriage going from {self._cur_column} to column {target_column}"
        )

        # Calculate the number of steps to take based on current position
        steps = HOME_TO_FIRST_COLUMN_DISTANCE_MM + target_column * STEPS_PER_COLUMN
        self._gcode_board.move('X', steps)
        self._cur_column = target_column

    def go_home(self) -> None:
        """Moves carriage towards home until limit switch pressed.
        Then sets current column to the HOME_COLUMN_VALUE
        """
        logging.info("Carriage going home.")
        self._gcode_board.move('X', 0)
        self._cur_column = HOME_COLUMN_VALUE


class Elevator:
    def __init__(self, gcode_board: GCodeBoard) -> None:
        self._gcode_board = gcode_board

    def push_one_ball(self):
        self._gcode_board.move_Y_one_rotation()

class MarbleMirror:
    def __init__(self, n_cols: int, n_rows: int) -> None:

        # Main class for the whole mirror.
        self._gcode_board = GCodeBoard(port="/dev/ttyUSB0")
        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        # self._elevator = StepperMotor(channel=ELEVATOR_STEPPER_CHANNEL, step_sleep = 0.005)
        self._elevator = Elevator(self._gcode_board)
        self._carriage = Carriage(self._gcode_board)
        self._board_dropper = Gate(
            open_angle=BOARD_SERVO_OPEN_ANGLE,
            closed_angle=BOARD_SERVO_CLOSE_ANGLE,
            channel=BOARD_SERVO_CHANNEL,
        )
        self._ball_reader = BallReader()

    def draw_image(self, image: List[List[int]]) -> None:

        """
        :param image: a list of lists of ints representing the image we want to draw. Each sublist is a column.
                        see MarbleBoard.set_new_board() for how they're organized.
        """

        # Get rid of any old image
        self.clear_image()

        # Set the new image that we'll be drawing
        self._board.set_new_board(image)
        self._carriage.go_home()
        while not self._board.done():
            # Push elevator until we have a ball in the cart
            cur_ball_color = self._ball_reader.color
            while cur_ball_color is BallState.Empty:
                logging.info("No current ball; pushing next ball...")
                # There is no current ball, push until we get one.
                self._elevator.push_one_ball()
                sleep(ELEVATOR_PUSH_WAIT_TIME_S)
                cur_ball_color = self._ball_reader.color

            # Get next column that we can drop this ball in
            valid_column = self._board.get_next_valid_column_for_color(cur_ball_color)

            # None corresponds to no columns need this color
            if valid_column is None:
                logging.info(
                    f"No column needs current ball ({cur_ball_color}); recycling"
                )
                self._carriage._ball_dropper.drop()
            else:
                logging.info(f"Delivering ball to col {valid_column}.")
                self._carriage.drop_ball_in_column_and_home(valid_column)
            self._board.print_board_queues()

    def clear_image(self) -> None:
        logging.debug("Clearing image.")
        self._board_dropper.drop(delay=BOARD_DROP_SLEEP_TIME)
        logging.debug("Image cleared.")


class Interface:
    pass


@click.group()
@click.option("--debug/--no-debug", default=False)
def cli(debug):
    if debug:
        loglevel = logging.DEBUG
    else:
        loglevel = logging.INFO
    logging.basicConfig(format="%(levelname)s:%(message)s", level=loglevel)


@cli.command()
@click.argument("column")
def goto(column):
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS)
    mm._carriage.go_to_column(int(column))
    logging.info(f"Arrived at column {column}")

@cli.command()
def lift():
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS)
    logging.info(f"Driving elevator one full rotation...")
    mm._elevator.move(ELEVATOR_BALL_PUSH_STEPS, ElevatorMoveDirection.BALL_UP)
    logging.info(f"Done driving elevator.")

@cli.command()
def drop():
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS)
    logging.info(f"Dropping carriage ball...")
    mm._carriage._ball_dropper.drop()
    logging.info(f"Dropper dropped.")

@cli.command()
def read():
    _ball_reader = BallReader()
    logging.info(f"Reading from sensor...")
    _ball_reader.color
    logging.info(f"Reading completed.")

@cli.command()
@click.argument("open_angle", default=BOARD_SERVO_OPEN_ANGLE)
@click.argument("close_angle", default=BOARD_SERVO_CLOSE_ANGLE)
def clear(open_angle, close_angle):
    _board_dropper = Gate(
            open_angle=int(open_angle),
            closed_angle=int(close_angle),
            channel=int(BOARD_SERVO_CHANNEL),
        )
    _board_dropper.drop()

@cli.command()
def draw():
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS)

    img = [
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
    ]
    logging.info("Drawing image...")
    mm.draw_image(img)

    logging.info("Drawing of image completed! Sleeping to appreciate art.")
    sleep(APPRECIATE_IMAGE_TIME)


if __name__ == "__main__":
    draw()
