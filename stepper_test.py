# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for using adafruit_motorkit with a stepper motor"""
import time
import board
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())

for i in range(100):
    kit.stepper1.onestep()
    time.sleep(0.01)

    #kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
    #kit.stepper1.onestep(style=stepper.MICROSTEP)