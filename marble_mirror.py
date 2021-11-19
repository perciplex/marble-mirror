from enum import IntEnum, Enum
from typing import List, Optional, Any
from collections import deque
from time import sleep
import logging
from marble_control import BallReader, BallReaderKNN, BallState, StepperMotor, LimitSwitch, Gate
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
HOME_COLUMN_VALUE = -1  # Doing this because we want the columns to be 0-indexed.
ELEVATOR_BALL_PUSH_STEPS_FULL_ROTATION = 202  # Set intentionally
ELEVATOR_BALL_PUSH_STEPS = ELEVATOR_BALL_PUSH_STEPS_FULL_ROTATION  # Incremental amount
CARRIAGE_MOTOR_COLUMN_STEPS = 325  # Set intentionally
HOME_TO_FIRST_COLUMN_ADDITIONAL_OFFSET_STEPS = 165  # Set intentionally; could be a bit more precise
ELEVATOR_STEP_SLEEP = 0.001  # Set
ELEVATOR_PUSH_WAIT_TIME_S = 1.5  # How long it waits after pushing a ball to do a reading, before pushing again


class ElevatorMoveDirection(IntEnum):
    BALL_UP = stepper.BACKWARD
    BALL_DOWN = stepper.FORWARD

class CarriageMoveDirection(IntEnum):
    AWAY = stepper.BACKWARD
    TOWARDS = stepper.FORWARD



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

        logging.error(f'Looking for valid column for ball color: = {ball_color}')
        # Go through each queue, see if the top one is the one we want. If it is, pop that marbel and return that number.
        for i, q in enumerate(self._queues_list):
            logging.error(f'\tchecking queue {i}...')
            if len(q) > 0:
                logging.error(f'\tnext color needed for this column is {q[0]}')
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

    def limit_is_pressed(self):
        return self._limit_switch.is_pressed

    def take_step(self, direction: int, style: Any) -> bool:
        """
        Returns the success status of taking the step: True if it actually took it, False if it didn't
        end up taking it.
        """
        sleep(ELEVATOR_STEP_SLEEP)
        # logging.error(f'Moving carriage motor in direction: {direction}')
        if direction == CarriageMoveDirection.AWAY:
            # If it's moving away, we don't care if the limit switch is being pressed, just take the step.
            step_result = super().take_step(direction=direction, style=style)
            # logging.error(f'Moving away, step_result = {step_result}')
            return True
        elif direction == CarriageMoveDirection.TOWARDS:
            if not self._limit_switch.is_pressed:
                step_result = super().take_step(direction=direction, style=style)
                # logging.error(f'Moving towards, switch not pressed. step_result = {step_result}')
                return True
            else:
                # logging.error(f'Moving towards, switch pressed. step_result = False')
                return False
        else:
            raise


class Carriage:
    def __init__(self) -> None:
        self._ball_dropper = Gate(
            open_angle=CARRIAGE_SERVO_OPEN_ANGLE,
            closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE,
            channel=CARRIAGE_SERVO_CHANNEL,
        )
        self._carriage_motor = CarriageMotor(channel=CARRIAGE_STEPPER_CHANNEL)
        # self._cur_column = None
        self._cur_column = HOME_COLUMN_VALUE
        self.go_home()

    def drop_ball_in_column_and_home(self, target_column: int) -> None:

        self.go_to_column(target_column)
        self.drop_ball()
        self.go_home()
        self._carriage_motor.release()

    def recycle_current_ball(self) -> None:
        assert self._cur_column == HOME_COLUMN_VALUE
        self.drop_ball()

    def drop_ball(self) -> None:
        self._ball_dropper.open()
        sleep(0.5)
        self._ball_dropper.close()

    def go_to_column(self, target_column: int) -> None:

        logging.error(f"Carriage going to column {target_column}")
        # We want *this* var to be positive when going away. If you're currently in column 2 and want
        # to go to column 5, the distance_to_move > 0.
        distance_to_move = int((target_column - self._cur_column) * STEPS_PER_COLUMN)

        self._carriage_motor.move(distance_to_move, CarriageMoveDirection.AWAY)
        # TODO: make sure this either blocks, or sleep some amount during drive

        self._cur_column = target_column

    def go_home(self) -> None:
        logging.error(f"Carriage going home, CarriageMoveDirection.TOWARDS = {CarriageMoveDirection.TOWARDS}")
        while not self._carriage_motor.limit_is_pressed():
            self._carriage_motor.move(HOME_MOVE_LARGE_AMOUNT, CarriageMoveDirection.TOWARDS)
        
        # This is to move it a *little* away from the "home" position, so that if you move N columns away,
        # it will actually go to them (and not have any offset).
        self._carriage_motor.move(HOME_TO_FIRST_COLUMN_ADDITIONAL_OFFSET_STEPS, CarriageMoveDirection.AWAY)
        # Carriage should now be right at limit switch trigger
        self._cur_column = HOME_COLUMN_VALUE


class Elevator(StepperMotor):
    def push_next_ball(self):
        super().move(ELEVATOR_BALL_PUSH_STEPS, ElevatorMoveDirection.BALL_UP)

    def take_step(self, direction: int, style: Any) -> bool:
        sleep(ELEVATOR_STEP_SLEEP)
        return super().take_step(direction=direction, style=style)


class MarbleMirror:
    def __init__(self, n_cols: int, n_rows: int) -> None:

        # Main class for the whole mirror.
        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        self._elevator = Elevator(channel=ELEVATOR_STEPPER_CHANNEL)
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

        logging.error("Clearing image")
        # Get rid of any old image
        self.clear_image()

        logging.error("Setting new image")
        # Set the new image that we'll be drawing
        self._board.set_new_board(image)

        while not self._board.done():

            logging.error("Reading current ball color")
            # Get current ball color
            current_ball_color = self._ball_reader.color

            while current_ball_color is BallState.Empty:
                logging.error("No current ball; pushing next ball")
                # If there's no current next ball, push so we hopefully have a next ball
                self._elevator.push_next_ball()
                sleep(ELEVATOR_PUSH_WAIT_TIME_S)
                logging.error("Reading current ball color")
                # Get current ball color
                current_ball_color = self._ball_reader.color



            logging.error("Getting next valid column for current ball\n")
            # Get next column that we can drop this ball in
            column_that_needs_current_ball = (
                self._board.get_next_valid_column_for_color(current_ball_color)
            )

            # None corresponds to no columns need this color
            if column_that_needs_current_ball is None:
                logging.error(f"No column needs current ball ({current_ball_color}); recycling")
                self._carriage.recycle_current_ball()
            else:
                logging.error(
                    f"Dropping ball in column {column_that_needs_current_ball}"
                )
                self._carriage.drop_ball_in_column_and_home(
                    column_that_needs_current_ball
                )

        # sleep in case there's another image up next, and you want to see this one for some period
        sleep(APPRECIATE_IMAGE_TIME)

    def clear_image(self) -> None:
        self._board_dropper.open()
        sleep(BOARD_DROP_SLEEP_TIME)
        self._board_dropper.close()


    def id_ball_loop(self):

        logging.error("Homing carriage")
        self._carriage.go_home()
        logging.error("Carriage homed")

        for i in range(100):

            self._elevator.push_next_ball()
            sleep(ELEVATOR_PUSH_WAIT_TIME_S)

            logging.error("\n\nReading current ball color")
            # Get current ball color
            logging.error(f"Ball color is: {self._ball_reader.color}")

            sleep(2.0)

            logging.error("Dropping ball")
            self._carriage.drop_ball()




if __name__ == "__main__":

    # TODO: make it close servos at the beginning!
    # TODO: why does the carriage move irregularly?
    # TODO: stepper at bottom too weak
    # TODO: get elevator wheel shape right
    # TODO: ball reader
    # fiugre out where ball reader goes
    # use plexiglas for column dividers
    # make whole thing bigger
    # rails, if doing that
    # move limit switch
    # make ball drop funnel thing narrower (basically one marble width)
    # remove dead space
    # maybe make recycling zone more tilted at end so balls don't get stuck
    # make connect 4 drop servo work (is it a weird velocity one?)
    # design in such a way that it can be scalable easily
    # 

    mm = MarbleMirror(n_cols=8, n_rows=8)
    # mm.id_ball_loop()

    # exit()
    # img = [[0, 1, 0, 1], [1, 0, 1, 0]]
    # img = [[0, 1], [0, 1], [1, 0], [1, 0]]

    # img = [[0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0],]
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

    # elevator = StepperMotor(channel=ELEVATOR_STEPPER_CHANNEL)
    # elevator.move(steps=100, direction=-1)

    # carriage_motor = CarriageMotor(channel=CARRIAGE_STEPPER_CHANNEL)
    # carriage_motor.move(steps=-100, direction=1)

    exit()
    carriage = Carriage()
    carriage.go_to_column(2)
    carriage.go_home()