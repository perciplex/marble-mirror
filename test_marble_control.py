import time

from marble_control import Gate, Stepper, LimitSwitch, Pixel
from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit

from adafruit_motor import stepper, servo


def test_dual_servo():
    servo = Gate()
    servo_2 = Gate(channel=1, closed_angle=100)
    servo.drop()
    servo_2.drop()

def test_stepper():
    stepper = Stepper()
    stepper.move(50)


def test_stepper_reverse():
    stepper = Stepper()
    stepper.move(-50)

def test_stepper_2():
    stepper = Stepper(channel=2)
    stepper.move(250)


def test_stepper_2_reverse():
    stepper = Stepper(channel=2)
    stepper.move(-250)

def test_switch_up():
    switch = LimitSwitch()
    assert switch.value

def test_switch_down():
    switch = LimitSwitch()
    for _ in range(5):
        print(switch.is_down)
        time.sleep(1)


def test_pixel_value():
    pixel = Pixel()
    print(pixel.value)