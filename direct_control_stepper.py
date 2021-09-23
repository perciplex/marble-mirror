import argparse
from test_marble_control import test_stepper, test_stepper_reverse, \
                    test_stepper_2, test_stepper_2_reverse, \
                    test_elevator_push_ball, test_carriage_one_column_away, test_carriage_one_column_towards

parser = argparse.ArgumentParser()
parser.add_argument("--reverse", action='store_true')
parser.add_argument("--elevator", action='store_true')
parser.add_argument("--carriage", action='store_true')

parser.add_argument("--push_one_ball", action='store_true')
parser.add_argument("--carriage_column_away", action='store_true')
parser.add_argument("--carriage_column_towards", action='store_true')

args = parser.parse_args()

if args.elevator:
    if args.reverse:
        test_stepper_2_reverse()
    else:
        test_stepper_2()

if args.carriage:
    if args.reverse:
        test_stepper_reverse()
    else:
        test_stepper()

if args.push_one_ball:
    test_elevator_push_ball()

if args.carriage_column_away:
    test_carriage_one_column_away()

if args.carriage_column_towards:
    test_carriage_one_column_towards()