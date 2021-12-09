import time
import logging

from marble_control import BallReader, Gate, StepperMotor, LimitSwitch
from marble_mirror import (
    CarriageMoveDirection,
    Elevator,
    ELEVATOR_BALL_PUSH_STEPS,
    ElevatorMoveDirection,
    CarriageMotor,
    CARRIAGE_MOTOR_COLUMN_STEPS,
    Carriage,
    ElevatorMoveDirection,
    CARRIAGE_SERVO_OPEN_ANGLE,
    CARRIAGE_SERVO_CLOSE_ANGLE,
    STEPS_PER_COLUMN,
)
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit

from adafruit_motor import stepper, servo


def test_dual_servo():
    servo = Gate()
    servo_2 = Gate(channel=1, closed_angle=100)
    servo.drop()
    servo_2.drop()


def test_stepper(move_amount=50):
    stepper = StepperMotor(channel=1)
    stepper.move(move_amount, CarriageMoveDirection.TOWARDS)


def test_stepper_reverse(move_amount=50):
    stepper = StepperMotor(channel=1)
    stepper.move(move_amount, CarriageMoveDirection.AWAY)


def test_stepper_2(move_amount=250):
    stepper = StepperMotor(channel=2)
    stepper.move(move_amount, ElevatorMoveDirection.BALL_UP)


def test_stepper_2_reverse(move_amount=250):
    stepper = StepperMotor(channel=2)
    stepper.move(move_amount, ElevatorMoveDirection.BALL_DOWN)


def test_elevator_push_ball():
    stepper = Elevator(channel=2)
    stepper.push_next_ball()


def test_carriage_one_column_away():
    stepper = CarriageMotor(channel=1)
    logging.error(
        f"Carriage moving away, CarriageMoveDirection.AWAY = {CarriageMoveDirection.AWAY}"
    )
    stepper.move(STEPS_PER_COLUMN, CarriageMoveDirection.AWAY)


def test_carriage_one_column_towards():
    stepper = CarriageMotor(channel=1)
    logging.error(
        f"Carriage moving towards, CarriageMoveDirection.TOWARDS = {CarriageMoveDirection.TOWARDS}"
    )
    stepper.move(STEPS_PER_COLUMN, CarriageMoveDirection.TOWARDS)


def test_carriage_home():
    carriage = Carriage()
    carriage.go_home()


def test_switch_value():
    switch = LimitSwitch()
    print(switch.is_pressed)


def test_switch_up():
    switch = LimitSwitch()
    assert switch.value


def test_switch_down():
    switch = LimitSwitch()
    for _ in range(5):
        print(switch.is_down)
        time.sleep(1)


def test_ball_reader_value():
    ball_reader = BallReader()
    print(ball_reader.color)


def test_carriage_open():
    servo = Gate(
        open_angle=CARRIAGE_SERVO_OPEN_ANGLE, closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE
    )
    servo.open()


def test_carriage_close():
    servo = Gate(
        open_angle=CARRIAGE_SERVO_OPEN_ANGLE, closed_angle=CARRIAGE_SERVO_CLOSE_ANGLE
    )
    servo.close()
