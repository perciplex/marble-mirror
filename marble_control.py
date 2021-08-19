from adafruit_motorkit import MotorKit


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
        



