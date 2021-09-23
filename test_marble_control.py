import time

from marble_control import BallReader, Gate, StepperMotor, LimitSwitch
from marble_mirror import CarriageMoveDirection, Elevator, \
                        ELEVATOR_BALL_PUSH_STEPS, ElevatorMoveDirection, \
                        CarriageMotor, CARRIAGE_MOTOR_COLUMN_STEPS
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit

from adafruit_motor import stepper, servo


def test_dual_servo():
    servo = Gate()
    servo_2 = Gate(channel=1, closed_angle=100)
    servo.drop()
    servo_2.drop()


def test_stepper():
    stepper = StepperMotor(channel=1)
    stepper.move(50, 1)

def test_stepper_reverse():
    stepper = StepperMotor(channel=1)
    stepper.move(50, -1)


def test_stepper_2():
    stepper = StepperMotor(channel=2)
    stepper.move(250, 1)

def test_stepper_2_reverse():
    stepper = StepperMotor(channel=2)
    stepper.move(250, -1)


def test_elevator_push_ball():
    stepper = Elevator(channel=2)
    stepper.push_next_ball()


def test_carriage_one_column_away():
    stepper = CarriageMotor(channel=1)
    stepper.move(CARRIAGE_MOTOR_COLUMN_STEPS, CarriageMoveDirection.AWAY)

def test_carriage_one_column_towards():
    stepper = CarriageMotor(channel=1)
    stepper.move(CARRIAGE_MOTOR_COLUMN_STEPS, CarriageMoveDirection.TOWARDS)

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
    print(ball_reader.value)