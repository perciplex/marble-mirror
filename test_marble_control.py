from marble_control import Servo, Stepper, StepperAdafruit
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper, servo


def test_dual_servo():
    servo = Servo()
    servo_2 = Servo(channel=1)
    servo.drop()
    servo_2.drop()


def test_stepper():
    stepper = Stepper()
    stepper.move(50)


def test_stepper_2():
    stepper = Stepper(channel=2)
    stepper.move(50, direction=-1)


def test_stepper_reverse():
    stepper = Stepper()
    stepper.move(50, direction=-1)


def test_adafruit_stepper():
    kit = MotorKit(address=0x6F)
    for i in range(200):
        kit.stepper1.onestep(style=stepper.DOUBLE, direction=stepper.FORWARD)
    kit.stepper1.release()


def test_adafruit_stepper_reverse():
    kit = MotorKit(address=0x6F)
    for i in range(200):
        kit.stepper1.onestep(style=stepper.DOUBLE, direction=stepper.BACKWARD)
    kit.stepper1.release()


def test_adafruit_stepper_new():
    stepper = StepperAdafruit(1)
    stepper.move(-200)
    stepper.move(400)
    stepper.move(-600)
    stepper.move(400)
