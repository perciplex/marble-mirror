from flask import Flask
import time
import board
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())
app = Flask(__name__)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

@app.route("/motor1")
def hello_world():
    for i in range(100):
        kit.stepper1.onestep()
        time.sleep(0.01)
