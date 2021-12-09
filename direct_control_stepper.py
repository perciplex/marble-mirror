import click
import logging
import argparse
from time import sleep
from test_marble_control import (
    test_stepper,
    test_stepper_reverse,
    test_stepper_2,
    test_stepper_2_reverse,
    test_elevator_push_ball,
    test_carriage_one_column_away,
    test_carriage_one_column_towards,
    test_ball_reader_value,
    test_carriage_open,
    test_carriage_close,
    test_carriage_home,
    test_switch_value,
    test_image_drop,
)

parser = argparse.ArgumentParser()
parser.add_argument("--reverse", action="store_true")
parser.add_argument("--elevator", action="store_true")
parser.add_argument("--carriage", action="store_true")
parser.add_argument("--move_amount", type=int, default=50)
parser.add_argument("--ball_reader", action="store_true")
parser.add_argument("--switch_value", action="store_true")

parser.add_argument("--push_one_ball", action="store_true")
parser.add_argument("--carriage_column_away", action="store_true")
parser.add_argument("--carriage_column_towards", action="store_true")
parser.add_argument("--carriage_home", action="store_true")
parser.add_argument("--carriage_servo_open", action="store_true")
parser.add_argument("--carriage_servo_close", action="store_true")
parser.add_argument("--carriage_servo_drop", action="store_true")
parser.add_argument("--carriage_columns_away", type=int, default=None)
parser.add_argument("--carriage_columns_towards", type=int, default=None)

args = parser.parse_args()

if args.elevator:
    if args.reverse:
        test_stepper_2_reverse(args.move_amount)
    else:
        test_stepper_2(args.move_amount)

if args.carriage:
    if args.reverse:
        test_stepper_reverse(args.move_amount)
    else:
        test_stepper(args.move_amount)

if args.carriage_home:
    test_carriage_home()

if args.push_one_ball:
    test_elevator_push_ball()

if args.carriage_column_away:
    test_carriage_one_column_away()

if args.carriage_column_towards:
    test_carriage_one_column_towards()

if args.ball_reader:
    test_ball_reader_value()

if args.carriage_servo_open:
    test_carriage_open()

if args.carriage_servo_close:
    test_carriage_close()

if args.carriage_servo_drop:
    test_carriage_open()
    sleep(1.0)
    test_carriage_close()

if args.switch_value:
    test_switch_value()

if args.carriage_columns_away is not None:
    for _ in range(args.carriage_columns_away):
        sleep(0.1)
        test_carriage_one_column_away()

if args.carriage_columns_towards is not None:
    for _ in range(args.carriage_columns_towards):
        sleep(0.1)
        test_carriage_one_column_towards()


@click.group()
@click.option("--debug/--no-debug", default=False)
def cli(debug):
    click.echo(f"Debug mode is {'on' if debug else 'off'}")
    if debug:
        loglevel = logging.DEBUG
    else:
        loglevel = logging.ERROR
    logging.basicConfig(format="%(levelname)s:%(message)s", level=loglevel)
    mm = MarbleMirror(1, 1)


@cli.command()  # @cli, not @click!
def draw_image():
    logging.info("Drawing image...")


@cli.command()  # @cli, not @click!
def push_ball():
    logging.info("Pushing ball")


@cli.command()  # @cli, not @click!
def go_home():
    logging.info("Going home")


@cli.command()  # @cli, not @click!
def goto_col():
    logging.info("Pushing ball")
