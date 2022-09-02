import logging
import numpy as np
import click

from marble_mirror.hardware.gate import PiGate
from marble_mirror.hardware.camera import PiCamera
from marble_mirror.hardware.gcode import PiGCodeBoard
from marble_mirror.mirror import MarbleMirror, Elevator

APPRECIATE_IMAGE_TIME = 5.0
CARRIAGE_SERVO_CHANNEL = 0
BOARD_SERVO_CHANNEL = 12
CARRIAGE_SERVO_OPEN_ANGLE = 40
CARRIAGE_SERVO_CLOSE_ANGLE = 150
BOARD_SERVO_OPEN_ANGLE = 110
BOARD_SERVO_CLOSE_ANGLE = 145

N_COLS = 30
N_ROWS = 32


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
    """
    Initialize a MarbleMirror and drive carriage to column.

    Args:
        column (int): Column to drive to.
    """
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS)
    mm._carriage.go_to_column(int(column))
    logging.info(f"Arrived at column {column}")


@cli.command()
def lift():
    """
    Initialize a PiGcodeBoard and life the elevator one ball.
    """
    gcode_board = PiGCodeBoard(port="/dev/ttyUSB0", home=True)
    elevator = Elevator(gcode_board)
    logging.info(f"Driving elevator one full rotation...")
    elevator.push_one_ball()
    logging.info(f"Done driving elevator.")


@cli.command()
def drop():
    """
    Initialize a PiGate on the carriage servo and drop one ball.
    """
    test_angle = 90
    ball_dropper = PiGate(
        open_angle=test_angle - 51,  # default 0
        closed_angle=test_angle + 41,  # default 15
        channel=CARRIAGE_SERVO_CHANNEL,
    )
    logging.info(f"Dropping carriage ball...")
    ball_dropper.drop()
    logging.info(f"Dropper dropped.")


@cli.command()
def jitter():
    """
    Initialize a PiGate on the carriage and jitter the dropper servo.
    """
    ball_dropper = PiGate(
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
    """
    Initialize a PiGate on the carriage, set the open angle, and open the servo.

    Args:
        angle (float): Angle to set the open position.
    """
    closed_angle = 130

    gate = PiGate(
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
    _board_dropper = PiGate(
        open_angle=int(open_angle),
        closed_angle=int(close_angle),
        channel=int(BOARD_SERVO_CHANNEL),
    )
    _board_dropper.drop(delay=1)


@cli.command()
@click.option("--sim", is_flag=True)
def draw(sim: bool):
    """
    Init a MarbleMirror object, set the image, and draw the image.
    If the `sim` flag is set, use the appropriate simulation classes and
    take no physical actions for all hardware (including PiCamera)

    Args:
        sim (bool): Flag to enable simulation mode.
    """
    if sim:
        config = "sim"
    else:
        config = "pi"

    print(config)
    mm = MarbleMirror(n_cols=N_COLS, n_rows=N_ROWS, config=config)

    img = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
        [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    ]

    img = np.array(img)
    img = img.astype(int).tolist()
    mm._board.set_new_board(img)
    logging.info("Drawing image...")
    mm.draw_image(img)

    logging.info("Drawing of image completed!")
    logging.info(f"Total recycles: {mm.total_balls_recycled}")
    logging.info(f"Total steps traveled: {mm.total_dist_traveled}")
    # sleep(APPRECIATE_IMAGE_TIME)


@cli.command()
def auto_calibrate():
    model_name = "garbus"
    # pass None as the path, so the Camera obj won't try to load it.
    mm = MarbleMirror(0, 0, model_pickle_path=None)
    mm.auto_calibrate(model_name=model_name)


@cli.command()
def camera_read():
    ball_reader = PiCamera()
    color = ball_reader.color_raw
    print(color.shape)
    print(color[:11])


if __name__ == "__main__":
    draw()
