
from typing import List, Optional, Any
from collections import deque
from time import sleep

from adafruit_motorkit import MotorKit
from adafruit_servokit import ServoKit
from adafruit_motor import stepper


BLACK = 'black'
WHITE = 'white'
COLORS = [BLACK, WHITE]

STEPS_PER_REV = 200. 
MM_PER_REV = 8.
INTER_COLUMN_DISTANCE = 13.7
STEPS_PER_COLUMN = int(INTER_COLUMN_DISTANCE * STEPS_PER_REV / MM_PER_REV)
APPRECIATE_IMAGE_TIME = 5.
BOARD_DROP_SLEEP_TIME = 5.
ELEVATOR_STEPPER_CHANNEL = 0
CARRIAGE_STEPPER_CHANNEL = 1
CARRIAGE_SERVO_CHANNEL = 1
RELEASE_SERVO_CHANNEL = 2
CARRIAGE_SERVO_OPEN_PWM = 0
CARRIAGE_SERVO_CLOSE_PWM = 0
RELEASE_SERVO_OPEN_PWM = 0
RELEASE_SERVO_CLOSE_PWM = 0
LIMIT_SWITCH_GPIO_PIN = 1
HOME_MOVE_LARGE_AMOUNT = -10
HOME_COLUMN_VALUE = -1  # Doing this because we want the columns to be 0-indexed.


class MarbleBoard:

    def __init__(self, n_cols: int, n_rows: int):
        
        self._n_cols = n_cols
        self._n_rows = n_rows
        
        self._queues_list = None


    def set_new_board(self, image: List[List[int]]) -> None:
        
        '''

        [[0, 1, 2, 3], [0, 1, 2, 3], ...]

        gives

        col_0:  col_1:  ...
        3       3
        2       2
        1       1
        0       0

        popleft() removes from the bottom.
        '''


        self._queues_list = []

        assert len(image) <= self._n_cols, f'Can only have up to {self._n_cols} columns'

        for image_col in image:
            assert len(image_col) <= self._n_rows, f'Can only have up to {self._n_rows} rows'

            self._queues_list.append(deque(image_col))
            

    def get_next_valid_column_for_color(self, ball_color: str) -> Optional[int]:

        assert not self.done(), "Can't ask for next valid column when image is done"

        assert ball_color in COLORS, f'{ball_color=} is not a valid color'

        # Go through each queue, see if the top one is the one we want. If it is, pop that marbel and return that number.
        for i, q in enumerate(self._queues_list):
            if len(q) > 0 and q[0] == ball_color:
                q.popleft()
                return i

        # If we didn't find any, return None
        return None


    def done(self) -> bool:
        return all([len(q) == 0 for q in self._queues_list])


class Gate:
    
    def __init__(self, open_pwm: int, closed_pwm: int, channel: int) -> None:
        self.servo_kit = ServoKit(channels=16)
        self.servo = self.servo_kit.servo[channel]
        self.open_pwm = open_pwm
        self.closed_pwm = closed_pwm

    def open(self):
        self.servo.pwm = self.open_pwm

    def close(self):
        self.servo.pwm = self.closed_pwm

    def drop(self, delay=1):
        self.open()
        sleep(delay)
        self.close()
        sleep(delay)


class StepperMotor:
    def __init__(self, channel: int) -> None:
        self.kit = MotorKit()
        self.stepper = getattr(self.kit, f"stepper{channel}")

    def take_step(self, direction: int, style: Any) -> bool:
        self.stepper.onestep(direction=direction, style=style)
        return True

    def move(self, steps: int, direction: int = 1) -> None:
        if steps < 0:
            steps = -steps
            direction = -1

        if direction == 1:
            direction = stepper.FORWARD
        elif direction == -1:
            direction = stepper.BACKWARD

        for _ in range(steps):
            step_success = self.take_step(direction=direction, style=stepper.DOUBLE)
            if not step_success:
                break

    def __del__(self):
        self.stepper.release()


class CarriageMotor(StepperMotor):

    def limit_is_triggered(self):
        raise NotImplementedError

    def take_step(self, direction: int, style: Any) -> bool:
        if not self.limit_is_triggered():
            super().take_step(direction=direction, style=style)
        else:
            return False


class Carriage:

    def __init__(self) -> None:
        self._ball_dropper = Gate(open_pwm=CARRIAGE_SERVO_OPEN_PWM, closed_pwm=CARRIAGE_SERVO_CLOSE_PWM, channel=CARRIAGE_SERVO_CHANNEL)
        self._carriage_motor = CarriageMotor(channel=CARRIAGE_STEPPER_CHANNEL)
        self._cur_column = None
        self.go_home()


    def drop_ball_in_column_and_home(self, target_column: int) -> None:

        self.go_to_column(target_column)
        self.drop_ball()
        self.go_home()


    def recycle_current_ball(self) -> None:
        assert self._cur_column == HOME_COLUMN_VALUE
        self.drop_ball()


    def drop_ball(self) -> None:
        self._ball_dropper.open()
        sleep(0.5)
        self._ball_dropper.close()


    def go_to_column(self, target_column: int) -> None:
        
        distance_to_move = (target_column - self._cur_column) * INTER_COLUMN_DISTANCE

        self._carriage_motor.move(distance_to_move)
        # TODO: make sure this either blocks, or sleep some amount during drive

        self._cur_column = target_column

    
    def go_home(self) -> None:
        while not self._carriage_motor.limit_is_triggered():
            self._carriage_motor.move(HOME_MOVE_LARGE_AMOUNT)
        # Carriage should now be right at limit switch trigger
        self._cur_column = HOME_COLUMN_VALUE



class BallReader:

    def __init__(self) -> None:
        # Class for whatever method we use to get the value of the next ball
        pass

    def get_current_ball_color(self) -> str:
        ball_color = None
        # Must return either BLACK, WHITE, or None (if there's no ball at all there)
        return ball_color


class MarbleMirror:

    def __init__(self, n_cols: int, n_rows: int) -> None:

        # Main class for the whole mirror.

        self._board = MarbleBoard(n_cols=n_cols, n_rows=n_rows)
        self._elevator = StepperMotor(channel=ELEVATOR_STEPPER_CHANNEL)
        self._carriage = Carriage()
        self._board_dropper = Gate(open_pwm=RELEASE_SERVO_OPEN_PWM, closed_pwm=RELEASE_SERVO_CLOSE_PWM, channel=RELEASE_SERVO_CHANNEL)
        self._ball_reader = BallReader()


    def draw_image(self, image: List[List[int]]) -> None:

        '''
        :param image: a list of lists of ints representing the image we want to draw. Each sublist is a column.
                        see MarbleBoard.set_new_board() for how they're organized.
        '''
        

        # Get rid of any old image
        self.clear_image()

        # Set the new image that we'll be drawing
        self._board.set_new_board(image)

        while not self._board.done():
            
            # Get current ball color
            current_ball_color = self._ball_reader.get_current_ball_color()

            while current_ball_color is None:
                # If there's no current next ball, push so we hopefully have a next ball
                self._elevator.push_next_ball()
                # Get current ball color
                current_ball_color = self._ball_reader.get_current_ball_color()

            # If there is a current next ball, push so it goes into the carriage
            self._elevator.push_next_ball()

            # Get next column that we can drop this ball in
            column_that_needs_current_ball = self._board.get_next_valid_column_for_color(current_ball_color)

            # None corresponds to no columns need this color
            if column_that_needs_current_ball is None:
                self._carriage.recycle_current_ball()    
            else:
                self._carriage.drop_ball_in_column_and_home(column_that_needs_current_ball)
            
            
        # sleep in case there's another image up next, and you want to see this one for some period
        sleep(APPRECIATE_IMAGE_TIME)


    def clear_image(self) -> None:
        self._board_dropper.open()
        sleep(BOARD_DROP_SLEEP_TIME)
        self._board_dropper.close()


if __name__ == '__main__':

    mm = MarbleMirror(n_cols=4, n_rows=2)
    
    