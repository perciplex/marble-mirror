from enum import IntEnum
from typing import List, Optional, Any
from collections import deque
from time import sleep
import logging
from marble_control import (
    BallReader,
    BallReaderKNN,
    BallState,
    StepperMotor,
    LimitSwitch,
    Gate,
)
from adafruit_motor import stepper

STEPS_PER_REV = 200.0
MM_PER_REV = 8.0
INTER_COLUMN_DISTANCE = 10.3  # (mm). Original (non rails) was 13.7
# INTER_COLUMN_DISTANCE = 40
STEPS_PER_COLUMN = int(INTER_COLUMN_DISTANCE * STEPS_PER_REV / MM_PER_REV)
APPRECIATE_IMAGE_TIME = 5.0
BOARD_DROP_SLEEP_TIME = 5.0
# The wire with a ziptie on it is for the elevator stepper. Don't switch these channels.
ELEVATOR_STEPPER_CHANNEL = 2
CARRIAGE_STEPPER_CHANNEL = 1
CARRIAGE_SERVO_CHANNEL = 0
RELEASE_SERVO_CHANNEL = 1
CARRIAGE_SERVO_OPEN_ANGLE = 140
CARRIAGE_SERVO_CLOSE_ANGLE = 120
RELEASE_SERVO_OPEN_ANGLE = 0
RELEASE_SERVO_CLOSE_ANGLE = 0
LIMIT_SWITCH_GPIO_PIN = 1
HOME_MOVE_LARGE_AMOUNT = 1000  # Arbitrary
HOME_COLUMN_VALUE = -1  # This needs to get calibrated to home offset from column 0
ELEVATOR_BALL_PUSH_STEPS_FULL_ROTATION = 202  # Set intentionally
ELEVATOR_BALL_PUSH_STEPS = ELEVATOR_BALL_PUSH_STEPS_FULL_ROTATION  # Incremental amount
CARRIAGE_MOTOR_COLUMN_STEPS = 325  # Set intentionally
HOME_TO_FIRST_COLUMN_ADDITIONAL_OFFSET_STEPS = (
    165  # Set intentionally; could be a bit more precise
)
ELEVATOR_STEP_SLEEP = 0.001  # Set
ELEVATOR_PUSH_WAIT_TIME_S = (
    1.5  # How long it waits after pushing a ball to do a reading, before pushing again
)


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

        self._queues_list = None

    def set_new_board(self, image: List[List[int]]) -> None:
        """
        [[0, 1, 2, 3], [0, 1, 2, 3], ...]

        gives

        col_0:  col_1:  ...
        3       3
        2       2
        1       1
        0       0

        popleft() removes from the bottom.
        """
        self._queues_list = []

        assert len(image) <= self._n_cols, f"Can only have up to {self._n_cols} columns"

        for image_col in image:
            assert (
                len(image_col) <= self._n_rows
            ), f"Can only have up to {self._n_rows} rows"

            self._queues_list.append(deque(image_col))

    def get_next_valid_column_for_color(self, ball_color: str) -> Optional[int]:

        assert not self.done(), "Can't ask for next valid column when image is done"

        logging.debug(f"Looking for valid column for ball color: = {ball_color}")
        # Go through each queue, see if the top one is the one we want. If it is, pop that marbel and return that number.
        for i, q in enumerate(self._queues_list):
            if len(q) > 0 and q[0] == ball_color.value:
                q.popleft()
                return i

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
            logging.info(
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

        self.go_home()

    def drop_ball_in_column_and_home(self, target_column: int) -> None:
        """Go to target column, drop the ball, and then return home.

        Args:
            target_column (int): Target column to drop ball in.
        """
        logging.info(f"Dropping ball in column {target_column}")
        self.go_to_column(target_column)
        self._ball_dropper.drop()
        self.go_home()

    def go_to_column(self, target_column: int) -> None:
        logging.info(
            f"Carriage going to column {target_column} from {self._cur_column}"
        )

        # Calculate the number of steps to take based on current position
        steps = int((target_column - self._cur_column) * STEPS_PER_COLUMN)

        if steps > 0:
            # Moving away from homee. Set dirrection.
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
        logging.info(
            f"Carriage going home moving in direction {CarriageMoveDirection.HOME}"
        )
        self._carriage_motor.move(HOME_MOVE_LARGE_AMOUNT, CarriageMoveDirection.HOME)
        self._cur_column = HOME_COLUMN_VALUE


class MarbleMirror:
    def __init__(self, n_cols: int, n_rows: int) -> None:

        # Main class for the whole mirror.
        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        self._elevator = StepperMotor(
            channel=ELEVATOR_STEPPER_CHANNEL, step_sleep=ELEVATOR_STEP_SLEEP
        )
        self._carriage = Carriage()
        self._board_dropper = Gate(
            open_angle=RELEASE_SERVO_OPEN_ANGLE,
            closed_angle=RELEASE_SERVO_CLOSE_ANGLE,
            channel=RELEASE_SERVO_CHANNEL,
        )
        self._ball_reader = BallReaderKNN()

    def draw_image(self, image: List[List[int]]) -> None:

        """
        :param image: a list of lists of ints representing the image we want to draw. Each sublist is a column.
                        see MarbleBoard.set_new_board() for how they're organized.
        """

        # Get rid of any old image
        self.clear_image()

        # Set the new image that we'll be drawing
        self._board.set_new_board(image)

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
                logging.info("Ball needed in column {valid_column}")
                self._carriage.drop_ball_in_column_and_home(valid_column)

    def clear_image(self) -> None:
        self._board_dropper.drop(delay=BOARD_DROP_SLEEP_TIME)


if __name__ == "__main__":
    mm = MarbleMirror(n_cols=8, n_rows=8)
    logging.basicConfig(format="%(levelname)s:%(message)s", level=logging.INFO)

    img = [
        [1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 1, 1, 1, 1, 1, 1],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0],
    ]
    mm.draw_image(img)

    # sleep in case there's another image up next, and you want to see this one for some period
    logging.info("Drawing of image completed! Sleeping to appreciate art.")
    sleep(APPRECIATE_IMAGE_TIME)
