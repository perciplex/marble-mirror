import logging
import numpy as np
import pickle
from dataclasses import dataclass
from sklearn.cluster import KMeans
from time import sleep
from typing import List, Type

from marble_mirror.hardware.camera import BallState, Camera, SimCamera, PiCamera
from marble_mirror.components import Elevator, Carriage, MarbleBoard
from marble_mirror.hardware.gate import Gate, SimGate, PiGate
from marble_mirror.hardware.gcode import GCodeBoard, SimGCodeBoard, PiGCodeBoard
from marble_mirror.planners import ClosestValidColumn, BalanceFrontier

BOARD_DROP_SLEEP_TIME = 1.0
CARRIAGE_SERVO_CHANNEL = 0
BOARD_SERVO_CHANNEL = 12
CARRIAGE_SERVO_OPEN_ANGLE = 40
CARRIAGE_SERVO_CLOSE_ANGLE = 150
BOARD_SERVO_OPEN_ANGLE = 110
BOARD_SERVO_CLOSE_ANGLE = 145


@dataclass
class MarbleMirrorHardwareFactory:
    camera_class: Type[Camera]
    gate_class: Type[Gate]
    gcode_class: Type[GCodeBoard]


HARDWARE_FACTORY_CONFIGS = {
    "sim": MarbleMirrorHardwareFactory(SimCamera, SimGate, SimGCodeBoard),
    "pi": MarbleMirrorHardwareFactory(PiCamera, PiGate, PiGCodeBoard),
}


def build_mm_hardware_factory(config: str) -> MarbleMirrorHardwareFactory:
    """
    Build the factory for the MarbleMirror hardward components.

    Args:
        config (str): Which config to use. Either "sim" or "pi" currently.

    Returns:
        MarbleMirrorFactory: Factory of MarbleMirror hardware components.
    """
    try:
        return HARDWARE_FACTORY_CONFIGS[config]
    except KeyError:
        logging.error(
            f"Unknown config provided. Only avail options are {HARDWARE_FACTORY_CONFIGS.keys()} and you provided {config}"
        )
        exit(1)


class MarbleMirror:
    def __init__(
        self, n_cols: int, n_rows: int, config: str, model_pickle_path="model_brig_weird_instruction.pickle"
    ) -> None:

        # Load in factory objects for the cgodeboard, camera, and servo gate. Then init them.
        factory = build_mm_hardware_factory(config)
        self._gcode_board = factory.gcode_class(port="/dev/ttyUSB0")
        self._ball_reader = factory.camera_class(model_pickle_path=model_pickle_path)
        self._board_dropper = factory.gate_class(
            open_angle=BOARD_SERVO_OPEN_ANGLE,
            closed_angle=BOARD_SERVO_CLOSE_ANGLE,
            channel=BOARD_SERVO_CHANNEL,
        )
        self._carriage_dropper = factory.gate_class(
            open_angle=CARRIAGE_SERVO_OPEN_ANGLE,
            closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE,
            channel=CARRIAGE_SERVO_CHANNEL,
        )

        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        self._elevator = Elevator(self._gcode_board)
        self._carriage = Carriage(self._gcode_board, self._carriage_dropper)
        self._planneer = ClosestValidColumn(self._board)
        self.total_balls_recycled = 0
        self.total_dist_traveled = 0

    def draw_image(self, image: List[List[int]]) -> None:
        """
        Draw the image provided on the MarbleMirror. Clear the old image,
        send the carriage home, load the new image into the board, drive the
        system until the image is completed.

        Args:
            image (List[List[int]]): A list of lists of ints representing the image we want to draw. Each sublist is a column.
                        see MarbleBoard.set_new_board() for how they're organized.
        """
        # Get rid of any old image
        self.clear_image()

        # Set the new image that we'll be drawing
        self._board.set_new_board(image)
        self._carriage.go_home()

        while not self._board.done():
            # Read in the current ball in carriage.
            cur_ball_color = self._ball_reader.color

            # While carriage is empty, go home and refill.
            while cur_ball_color is BallState.Empty:
                logging.info("No current ball; homing and refilling")
                # There is no current ball, go home and fill carriage.
                self._carriage.go_home()
                self._elevator.fill_carriage()
                cur_ball_color = self._ball_reader.color

            target_col = self._planneer.get_drop_column(
                cur_ball_color=cur_ball_color, cur_cart_column=self._carriage._cur_column
            )

            # None corresponds to no columns need this color
            if target_col is None:
                logging.info(f"No column needs current ball ({cur_ball_color}); recycling")
                self.total_dist_traveled += self._carriage.go_home()
                self._carriage._ball_dropper.drop()
                self.total_balls_recycled += 1
            else:
                logging.info(f"Delivering ball to col {target_col}.")
                assert BallState(self._board._queues_list[target_col][-1]) == cur_ball_color
                self._board._queues_list[target_col].pop()
                self.total_dist_traveled += self._carriage.drop_ball_in_column(target_col)

            self._board.print_board_queues()

    def clear_image(self) -> None:
        logging.debug("Clearing image.")
        self._board_dropper.drop(delay=BOARD_DROP_SLEEP_TIME)
        logging.debug("Image cleared.")

    # TODO: Format this better
    def auto_calibrate(self, model_name: str):
        # pass None as the path, so the Camera obj won't try to load it.
        # mm = MarbleMirror(1, 1, model_pickle_path=None)
        car = self._carriage
        # ball = mm._ball_reader.pixel
        ball = self._ball_reader
        elevator = self._elevator

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
            for _ in range(30):
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


if __name__ == "__main__":
    mm = MarbleMirror(n_cols=32, n_rows=32, config="sim")
    img = []
    mm.draw_image(image=img)
