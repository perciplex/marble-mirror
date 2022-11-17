import logging
import numpy as np
from enum import IntEnum
from typing import List
from adafruit_motor import stepper

from marble_mirror.hardware.camera import BallState
from marble_mirror.hardware.gate import Gate
from marble_mirror.hardware.gcode import GCodeBoard


STEPS_PER_COLUMN = 7.044
HOME_COLUMN_VALUE = 0.0
HOME_TO_FIRST_COLUMN_DISTANCE_MM = 33  # This needs to get calibrated to home offset from column 0
CARRIAGE_CAPACITY = 5
BALLS_PER_PACMAN_ROTATION = 3


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
        logging.debug(f"Current state of internal queues: {[_ for _ in self._queues_list]}")
        for row in self._board_state:
            logging.debug(row)

    def get_frontier_at_depth(self, depth: int = 0) -> List[BallState]:
        """
        Return a frontier of the queues at a fixed depth. Depth 0 is the current frontier.

        Args:
            depth (int, optional): Frontier depth. Defaults to 0.

        Returns:
            List[BallState]: Frontier at requested depth.
        """
        adj_depth = -1 - depth
        frontier = []

        for queue in self._queues_list:
            if len(queue) >= abs(adj_depth):
                frontier.append(BallState(queue[adj_depth]))
            else:
                frontier.append(BallState.Empty)

        logging.debug(f"Frontier at depth {depth}: {frontier}")
        return frontier

    def done(self) -> bool:
        return all([len(q) == 0 for q in self._queues_list])


class Carriage:
    """
    Carriage class which is repsonsible for reading bals and dropping them
    in the target columns. Has a GCodeBoard for driving and a Gate for droping.
    """

    def __init__(self, gcode_board: GCodeBoard, gate: Gate) -> None:
        self._ball_dropper = gate
        self._gcode_board = gcode_board
        self._cur_column = 0

    def calc_position(self, column) -> float:
        """
        Calculate the GRBL position based on given column.

        Returns:
            float: Position in steps of provided column.
        """
        return HOME_TO_FIRST_COLUMN_DISTANCE_MM + column * STEPS_PER_COLUMN

    def drop_ball_in_column(self, target_column: int) -> float:
        """Go to target column, drop the ball, and then return home.

        Args:
            target_column (int): Target column to drop ball in.

        Returns:
            int: total steps traveled
        """
        logging.debug(f"Dropping ball in column {target_column}")
        steps = self.go_to_column(target_column)
        self._ball_dropper.drop()
        return steps

    def go_to_column(self, target_column: int) -> float:
        """Move the carriage to the target column. Calculates the inter-column
        distance, converts to stepper steps, and determines the direction.

        If the current column is not known, first go home to calibrate.

        Returns:
            float: Total steps traveled.

        Args:
            target_column (int): The target column to go to.
        """

        logging.debug(f"Carriage going from {self._cur_column} to column {target_column}")

        # Calculate the absolute positions of current and target columns
        current_pos = self.calc_position(self._cur_column)
        target_pos = self.calc_position(target_column)

        # Move and set new column.
        self._gcode_board.move("X", target_pos)
        self._cur_column = target_column

        return abs(target_pos - current_pos)

    def go_home(self) -> float:
        """Moves carriage towards home until limit switch pressed, then sets current
        column to the HOME_COLUMN_VALUE. Returns the estimated distance traveleled.

        Returns:
            float: Total steps traveled.
        """
        logging.info("Carriage going home.")

        # Calculate the absolute positions of current and target columns
        current_pos = self.calc_position(self._cur_column)
        target_pos = self.calc_position(HOME_COLUMN_VALUE)

        self._gcode_board.move("X", 0)
        self._cur_column = HOME_COLUMN_VALUE

        return abs(target_pos - current_pos)


class Elevator:
    """
    Elevator class responsible for driving balls up to carriage.
    Has a GCodeBoard for driving the stepper.
    """

    def __init__(self, gcode_board: GCodeBoard) -> None:
        self._gcode_board = gcode_board

    def push_one_ball(self):
        self._gcode_board.move_Y_n_rotation(1.0 / BALLS_PER_PACMAN_ROTATION)

    def fill_carriage(self):
        self._gcode_board.move_Y_n_rotation(CARRIAGE_CAPACITY / BALLS_PER_PACMAN_ROTATION)
