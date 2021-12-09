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
)

STEPS_PER_REV = 200.0
MM_PER_REV = 8.0
INTER_COLUMN_DISTANCE = 10.3  # (mm). Original (non rails) was 13.7
STEPS_PER_COLUMN = int(INTER_COLUMN_DISTANCE * STEPS_PER_REV / MM_PER_REV)
APPRECIATE_IMAGE_TIME = 5.0
BOARD_DROP_SLEEP_TIME = 1.0
# The wire with a ziptie on it is for the elevator stepper.
ELEVATOR_STEPPER_CHANNEL = 2
CARRIAGE_STEPPER_CHANNEL = 1
CARRIAGE_SERVO_CHANNEL = 0
BOARD_SERVO_CHANNEL = 1
CARRIAGE_SERVO_OPEN_ANGLE = 140
CARRIAGE_SERVO_CLOSE_ANGLE = 120
BOARD_SERVO_OPEN_ANGLE = 0
BOARD_SERVO_CLOSE_ANGLE = 0
LIMIT_SWITCH_GPIO_PIN = 1
HOME_COLUMN_VALUE = -1  # This needs to get calibrated to home offset from column 0
ELEVATOR_BALL_PUSH_STEPS = 202  # Set intentionally
ELEVATOR_PUSH_WAIT_TIME_S = 1.5  # Wait after pushing a ball before reading

N_COLS = 8
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
        logging.error(
            f"Current state of internal queues: {[_ for _ in self._queues_list]}"
        )
        # print(*(' '.join(row) for row in self._board_state), sep='\n')
        # logging.error(f"Current expected board state: {np.array(self._board_state)}")
        for row in self._board_state:
            print(*row, sep=", ")

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


class CarriageMotor(StepperMotor):
    def __init__(self, channel: int):
        self._limit_switch = LimitSwitch()
        super().__init__(channel=channel)

    def can_step(self, direction):
        if direction == CarriageMoveDirection.HOME and self._limit_switch.is_pressed:
            logging.debug(
                f"Cant take step in direction {direction}, limit switch is pressed!"
            )
            return False
        else:
            return True


class Carriage:
    def __init__(self) -> None:
        self._ball_dropper = Gate(
            open_angle=CARRIAGE_SERVO_OPEN_ANGLE,
            closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE,
            channel=CARRIAGE_SERVO_CHANNEL,
        )
        self._carriage_motor = CarriageMotor(channel=CARRIAGE_STEPPER_CHANNEL)
        self._cur_column = None

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
        if not self._cur_column:
            self.go_home()

        logging.debug(
            f"Carriage going from {self._cur_column} to column {target_column}"
        )

        # Calculate the number of steps to take based on current position
        steps = int((target_column - self._cur_column) * STEPS_PER_COLUMN)

        if steps > 0:
            # Moving away from home. Set direction.
            direction = CarriageMoveDirection.AWAY
        else:
            # Moving towards home. Set direction and abs(steps)
            direction = CarriageMoveDirection.HOME
            steps = abs(steps)

        self._carriage_motor.move(steps, direction)
        self._cur_column = target_column

    def go_home(self) -> None:
        """Moves carriage towards home until limit switch pressed.
        Then sets current column to the HOME_COLUMN_VALUE
        """
        logging.info("Carriage going home.")
        # While the carriage can still move towards home, keep moving.
        while self._carriage_motor.can_step(CarriageMoveDirection.HOME):
            self._carriage_motor.move(100, CarriageMoveDirection.HOME)
        self._cur_column = HOME_COLUMN_VALUE


class MarbleMirror:
    def __init__(self, n_cols: int, n_rows: int) -> None:

        # Main class for the whole mirror.
        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        self._elevator = StepperMotor(channel=ELEVATOR_STEPPER_CHANNEL)
        self._carriage = Carriage()
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
                self._elevator.move(
                    ELEVATOR_BALL_PUSH_STEPS, ElevatorMoveDirection.BALL_UP
                )
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
def test():
    pass


@cli.command()
def draw():
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS)

    img = [[1, 1], [0, 0]]
    # img = [
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 0, 0, 0, 0, 0, 0, 1],
    #     [0, 0, 1, 1, 1, 1, 1, 1],
    #     [0, 0, 1, 0, 0, 0, 0, 0],
    #     [0, 0, 1, 1, 1, 1, 0, 0],
    #     [0, 0, 0, 0, 0, 1, 0, 0],
    #     [0, 0, 0, 1, 1, 1, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    # ]
    logging.info("Drawing image...")
    mm.draw_image(img)

    logging.info("Drawing of image completed! Sleeping to appreciate art.")
    sleep(APPRECIATE_IMAGE_TIME)


if __name__ == "__main__":
    draw()
