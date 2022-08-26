import logging
import numpy as np
from enum import IntEnum
from time import sleep
from typing import List, Optional, Type
import numpy as np
import click
from adafruit_motor import stepper
from dataclasses import dataclass

from marble_control import BallState, Gate, SimGate, Camera, SimCamera, PiCamera, PiGate
from gcode import GCodeBoard, SimGCodeBoard, PiGCodeBoard
import pickle
from sklearn.cluster import KMeans

STEPS_PER_REV = 200.0
MM_PER_REV = 8.0
INTER_COLUMN_DISTANCE = 11.613  # (mm). Original (non rails) was 13.7
STEPS_PER_COLUMN = 7.044
APPRECIATE_IMAGE_TIME = 5.0
BOARD_DROP_SLEEP_TIME = 1.0
# The wire with a ziptie on it is for the elevator stepper.
ELEVATOR_STEPPER_CHANNEL = 2
CARRIAGE_STEPPER_CHANNEL = 1
CARRIAGE_SERVO_CHANNEL = 0
BOARD_SERVO_CHANNEL = 12
CARRIAGE_SERVO_OPEN_ANGLE = 40
CARRIAGE_SERVO_CLOSE_ANGLE = 150
BOARD_SERVO_OPEN_ANGLE = 110
BOARD_SERVO_CLOSE_ANGLE = 145
LIMIT_SWITCH_GPIO_PIN = 1
HOME_COLUMN_VALUE = 0.0  # This needs to get calibrated to home offset from column 0
ELEVATOR_BALL_PUSH_STEPS = 67  # Set intentionally
ELEVATOR_PUSH_WAIT_TIME_S = 0.5  # Wait after pushing a ball before reading
HOME_TO_FIRST_COLUMN_DISTANCE_MM = 33
CARRIAGE_CAPACITY = 5
BALLS_PER_PACMAN_ROTATION = 3

N_COLS = 30
N_ROWS = 32


class ElevatorMoveDirection(IntEnum):
    BALL_UP = stepper.BACKWARD
    BALL_DOWN = stepper.FORWARD


class CarriageMoveDirection(IntEnum):
    AWAY = stepper.BACKWARD
    HOME = stepper.FORWARD


def column_valid_for_color(col: List[int], ball_color: int):
    return len(col) and col[-1] == ball_color.value


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

    def get_closest_valid_column_for_color(self, ball_color: int, current_col: int) -> Optional[int]:
        logging.debug(f"Looking for closest valid column for ball color: = {ball_color}")
        valid_columns = [
            col for (col, queue) in enumerate(self._queues_list) if column_valid_for_color(queue, ball_color)
        ]
        if valid_columns:
            target_col = min(valid_columns, key=lambda col: abs(col - current_col))
            self._queues_list[target_col].pop()
            return target_col
        # If we didn't find any, return None
        return None

    def get_valid_column_for_color_and_minimize_diff(self, ball_color: int, current_col: int) -> Optional[int]:
        logging.debug(f"Looking for column for ball color: = {ball_color}")
        valid_columns = [
            col for (col, queue) in enumerate(self._queues_list) if column_valid_for_color(queue, ball_color)
        ]

        # If we didn't find any, return None
        if len(valid_columns) == 0:
            return None

        frontier_balls = [queue[-1] for queue in self._queues_list if len(queue)]
        n_white_balls = sum([b for b in frontier_balls if b == BallState.White.value])
        n_black_balls = sum([b for b in frontier_balls if b == BallState.Black.value])

        ball_most_needed = (
            ball_color
            if (n_black_balls == n_white_balls)
            else (BallState.Black.value if n_black_balls < n_white_balls else BallState.White.value)
        )

        # find the columns that have the ball we most want after the bottom one
        frontier_improving_cols = []
        for col in valid_columns:
            queue = self._queues_list[col]
            if len(queue) >= 2 and queue[-2] == ball_most_needed:
                frontier_improving_cols.append(col)

        # if they exist, find the closest one
        if frontier_improving_cols:
            target_col = min(frontier_improving_cols, key=lambda col: abs(col - current_col))
        else:
            target_col = min(valid_columns, key=lambda col: abs(col - current_col))

        self._queues_list[target_col].pop()
        return target_col

    def done(self) -> bool:
        return all([len(q) == 0 for q in self._queues_list])


class Carriage:
    def __init__(self, gcode_board: GCodeBoard) -> None:
        self._ball_dropper = SimGate(
            open_angle=CARRIAGE_SERVO_OPEN_ANGLE,
            closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE,
            channel=CARRIAGE_SERVO_CHANNEL,
        )
        self._gcode_board = gcode_board
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

        # Calculate the number of steps to take based on current position
        steps = HOME_TO_FIRST_COLUMN_DISTANCE_MM + target_column * STEPS_PER_COLUMN
        self._gcode_board.move("X", steps)
        self._cur_column = target_column
        return steps

    def go_home(self) -> None:
        """Moves carriage towards home until limit switch pressed.
        Then sets current column to the HOME_COLUMN_VALUE
        """
        logging.info("Carriage going home.")
        self._gcode_board.move("X", 0)
        self._cur_column = HOME_COLUMN_VALUE


class Elevator:
    def __init__(self, gcode_board: GCodeBoard) -> None:
        self._gcode_board = gcode_board

    def push_one_ball(self):
        self._gcode_board.move_Y_n_rotation(1.0 / BALLS_PER_PACMAN_ROTATION)

    def fill_carriage(self):
        self._gcode_board.move_Y_n_rotation(CARRIAGE_CAPACITY / BALLS_PER_PACMAN_ROTATION)


@dataclass
class MarbleMirrorFactory:
    camera_class: Type[Camera]
    gate_class: Type[Gate]
    gcode_class: Type[GCodeBoard]

FACTORY_CONFIGS = {
    "sim": MarbleMirrorFactory(SimCamera, SimGate, SimGCodeBoard),
    "pi": MarbleMirrorFactory(PiCamera, PiGate, PiGCodeBoard),
}


def build_factory(config: str) -> MarbleMirrorFactory:
    try:
        return FACTORY_CONFIGS[config]
    except KeyError:
        logging.error(
            f"Unknown config provided. Only avail options are {FACTORY_CONFIGS.keys()} and you provided {config}"
        )
        exit(1)


class MarbleMirror:
    def __init__(
        self, n_cols: int, n_rows: int, config="pi", model_pickle_path="model_brig_weird_instruction.pickle"
    ) -> None:
        factory = build_factory(config)
        # Load in factory objects for the cgodeboard, camera, and servo gate.
        self._gcode_board = factory.gcode_class(port="/dev/ttyUSB0")  # marble_mirror_objs.gcode_board
        self._ball_reader = factory.camera_class(model_pickle_path=model_pickle_path)
        self._board_dropper = factory.gate_class(
            open_angle=BOARD_SERVO_OPEN_ANGLE,
            closed_angle=BOARD_SERVO_CLOSE_ANGLE,
            channel=BOARD_SERVO_CHANNEL,
        )

        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        self._elevator = Elevator(self._gcode_board)
        self._carriage = Carriage(self._gcode_board)
        self.total_balls_recycled = 0
        self.total_dist_traveled = 0

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
                logging.info("No current ball; homing and refilling")
                # There is no current ball, push until we get one.
                self._carriage.go_home()
                # self._carriage._ball_dropper.drop()
                self._elevator.fill_carriage()
                cur_ball_color = self._ball_reader.color

            # TODO: Add a class here that will determine which col to drop in so we can 
            # experiment with different heuristics later on.
            # Get next column that we can drop this ball in
            valid_column = self._board.get_valid_column_for_color_and_minimize_diff(
                cur_ball_color, self._carriage._cur_column
            )

            # None corresponds to no columns need this color
            if valid_column is None:
                logging.info(f"No column needs current ball ({cur_ball_color}); recycling")
                self._carriage.go_home()
                self._carriage._ball_dropper.drop()
                self.total_balls_recycled += 1
            else:
                logging.info(f"Delivering ball to col {valid_column}.")
                self.total_dist_traveled += self._carriage.drop_ball_in_column(valid_column)

            self._board.print_board_queues()

    def clear_image(self) -> None:
        logging.debug("Clearing image.")
        self._board_dropper.drop(delay=BOARD_DROP_SLEEP_TIME)
        logging.debug("Image cleared.")


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
    gcode_board = GCodeBoard(port="/dev/ttyUSB0", home=True)
    elevator = Elevator(gcode_board)
    logging.info(f"Driving elevator one full rotation...")
    elevator.push_one_ball()
    logging.info(f"Done driving elevator.")


@cli.command()
def drop():
    test_angle = 90
    ball_dropper = Gate(
        open_angle=test_angle - 50,  # default 0
        closed_angle=test_angle + 40,  # default 15
        channel=CARRIAGE_SERVO_CHANNEL,
    )
    logging.info(f"Dropping carriage ball...")
    ball_dropper.drop(delay=0.5)
    logging.info(f"Dropper dropped.")


@cli.command()
def jitter():
    test_angle = 90
    ball_dropper = Gate(
        open_angle=CARRIAGE_SERVO_OPEN_ANGLE,  # default 0
        closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE,  # default 15
        channel=CARRIAGE_SERVO_CHANNEL,
    )
    logging.info(f"Dropping carriage ball...")
    ball_dropper.jitter()
    logging.info(f"Dropper dropped.")


@cli.command()
@click.argument("angle")
def set_angle(angle):

    # for reference
    # test_angle = 90
    # open_angle = 40
    # closed_angle = 130

    gate = Gate(
        open_angle=angle,
        closed_angle=closed_angle,
        channel=CARRIAGE_SERVO_CHANNEL,
    )
    logging.info(f"Setting servo angle to {angle}...")
    gate.open()
    logging.info(f"Set.")


@cli.command()
@click.argument("open_angle", default=BOARD_SERVO_OPEN_ANGLE)
@click.argument("close_angle", default=BOARD_SERVO_CLOSE_ANGLE)
def clear(open_angle, close_angle):
    _board_dropper = Gate(
        open_angle=int(open_angle),
        closed_angle=int(close_angle),
        channel=int(BOARD_SERVO_CHANNEL),
    )
    _board_dropper.drop(delay=1)


@cli.command()
@click.option('--sim', is_flag=True)
def draw(sim):
    if sim:
        config = "sim"
    else:
        config = "pi"

    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS, config=config)

    img = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    ]

    img = np.array(img)
    img = 1 - img
    img = img.astype(int).tolist()

    logging.info("Drawing image...")
    mm.draw_image(img)

    logging.info("Drawing of image completed!")
    logging.info(f"Total recycles: {mm.total_balls_recycled}")
    logging.info(f"Total steps traveled: {mm.total_dist_traveled}")
    # sleep(APPRECIATE_IMAGE_TIME)


@cli.command()
def auto_calibrate():
    # pass None as the path, so the Camera obj won't try to load it.
    mm = MarbleMirror(1, 1, model_pickle_path=None)
    car = mm._carriage
    # ball = mm._ball_reader.pixel
    ball = mm._ball_reader
    elevator = mm._elevator

    car.go_home()
    logging.info("Auto calibrating...")

    def measure(t=1, drop=True, push=True):
        if drop:
            car._ball_dropper.drop()
        if push:
            elevator.push_one_ball()
        sleep(t)
        data = None

        while data is None:
            data = ball.color_raw

        print("data array sum = {:.4f}".format(np.sum(data)))

        return data

    def get_labels_to_colors(kmeans_model, kmeans_data):

        black_label = np.argmax(np.linalg.norm(kmeans_model.cluster_centers_, axis=1))
        empty_label = kmeans_model.predict(kmeans_data[0:1])[0]
        white_label = [i for i in range(3) if i not in [black_label, empty_label]][0]
        return {
            white_label: "white",
            empty_label: "empty",
            black_label: "black",
        }

    def collect_calibration_data():

        all_data = []
        # definitely empty it
        logging.info("\nEmptying...")
        for _ in range(10):
            car._ball_dropper.drop()
        logging.info("\nEmpty:")
        empty_data = [measure(push=False) for _ in range(5)]  # read empty
        all_data += empty_data
        logging.info("\nBalls:")
        non_empty_data = [measure(push=True) for _ in range(15)]  # read ballz
        all_data += non_empty_data

        return empty_data, non_empty_data, all_data

    def collect_data_fit_model():
        logging.info("\nCollecting data...")
        empty_data, non_empty_data, all_data = collect_calibration_data()

        logging.info("\nDone, fitting model...")
        kmeans = KMeans(n_clusters=3, random_state=666).fit(all_data)
        logging.info("\nLabels of data; does this look right?\n")
        logging.info(kmeans.labels_)

        labels_to_colors = get_labels_to_colors(kmeans, all_data)
        logging.info("\nLabels to colors:")
        [logging.info(f"{k} = {v}") for k, v in labels_to_colors.items()]

        return kmeans, labels_to_colors

    def create_model_and_predict(model_name):

        kmeans, labels_to_colors = collect_data_fit_model()
        logging.info("\nPredicting with trained model now...")
        for i in range(30):
            datum = measure(push=True)
            pred = kmeans.predict([datum])[0]
            score = kmeans.score([datum])
            logging.info("Data = {},\tColor = {},\tScore = {:.3f}".format(datum, labels_to_colors[pred], score))

        # Dump model and vocab
        d = {"vocab": labels_to_colors, "model": kmeans}
        with open(f"{model_name}.pickle", "wb") as handle:
            pickle.dump(d, handle)

    model_name = "model_brig_weird_instruction"
    create_model_and_predict(model_name)
    logging.info(f"Wrote auto calibrated model to {model_name}")


@cli.command()
def camera_read():
    ball_reader = Camera()
    color = ball_reader.color_raw
    print(color.shape)
    print(color[:10])


if __name__ == "__main__":
    draw()
