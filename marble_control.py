import logging
from time import sleep
import atexit

from Raspi_MotorHAT.Raspi_PWM_Servo_Driver import PWM
from Raspi_MotorHAT import Raspi_MotorHAT

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper


ADDRESS = 0x6F


class Servo:
    def __init__(self, channel=0, min_pwm=200, max_pwm=400):
        logging.info(f"Setting up servo at address {ADDRESS}")
        self.pwm = PWM(ADDRESS)
        self.pwm.setPWMFreq(60)  # Set frequency to 60 Hz
        self.channel = channel

        self.min_pwm = min_pwm
        self.max_pwm = max_pwm

    def open(self):
        self.goto(self.max_pwm)

    def close(self):
        self.goto(self.min_pwm)

    def goto(self, pos):
        self.pwm.setPWM(self.channel, 0, pos)

    def drop(self, delay=1):
        self.open()
        sleep(delay)
        self.close()
        sleep(delay)


class Stepper:
    motor_channels = {1: (1, 2), 2: (3, 4)}

    def __init__(self, channel=1):
        self.motor_hat = Raspi_MotorHAT(ADDRESS, freq=800)
        self.stepper = self.motor_hat.getStepper(200, channel)
        self.stepper.setSpeed(60)
        self.channel = channel

    def move(self, steps, direction=Raspi_MotorHAT.FORWARD):
        if direction == 1:
            direction = Raspi_MotorHAT.FORWARD
        elif direction == -1:
            direction = Raspi_MotorHAT.BACKWARD

        self.stepper.step(steps, direction, Raspi_MotorHAT.DOUBLE)

    def __del__(self):
        for channel in self.motor_channels[self.channel]:
            self.motor_hat.getMotor(channel).run(Raspi_MotorHAT.RELEASE)


class StepperAdafruit:
    def __init__(self, channel=1):
        self.kit = MotorKit(address=0x6F)
        self.stepper = getattr(self.kit, f"stepper{channel}")

    def move(self, steps, direction=1):
        if steps < 0:
            steps = -steps
            direction = -1

        if direction == 1:
            direction = stepper.FORWARD
        elif direction == -1:
            direction = stepper.BACKWARD

        for i in range(steps):
            self.stepper.onestep(direction=direction, style=stepper.DOUBLE)

    def __del__(self):
        self.stepper.release()


"""
class MarbleMirror():
    columnoffset = 100
    col_steps = 10
    def __init__(self):
        self.step = 0
        self.kit = MotorKit(i2c=board.I2C())
    def goto_col(self, col):
        goalstep = columnoffset + col*col_steps
        dstep = self.step - goalstep
        if dstep > 0:
            for i in range(dstep):
                self.step += 1
                self.kit.stepper1.onestep(direction=stepper.FORWARD)
        if dstep < 0:
            for i in range(abs(dstep)):
                self.step -= 1
                self.kit.stepper1.onestep(direction=stepper.BACKWARD)
    def home(self):
        while self.step > 0:
            self.step -= 1
            self.kit.stepper1.onestep(direction=stepper.BACKWARD)
    def drop(self):
        self.kit.
"""
