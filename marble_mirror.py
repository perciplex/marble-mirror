
from typing import List, Optional
from collections import deque
from time import sleep


BLACK = 'black'
WHITE = 'white'
COLORS = [BLACK, WHITE]

STEPS_PER_REV = 200. 
MM_PER_REV = 8.
INTER_COLUMN_DISTANCE = 13.7
STEPS_PER_COLUMN = int(INTER_COLUMN_DISTANCE * STEPS_PER_REV / MM_PER_REV)
APPRECIATE_IMAGE_TIME = 5.
BOARD_DROP_SLEEP_TIME = 5.


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
    
    def __init__(self, open_pwm: int, closed_pwm: int) -> None:

        # Parent class for servos

        self._open_pwm = open_pwm
        self._closed_pwm = closed_pwm

    
    def open(self) -> None:
        pass
    
    def close(self) -> None:
        pass


class StepperMotor:
    pass


class CarriageMotor(StepperMotor):

    def __init__(self) -> None:

        # Class for the stepper motor that moves the carriage

        self._motor = None



class Elevator(StepperMotor):

    def __init__(self) -> None:

        # Class for the stepper motor that moves balls from the bottom to top

        self._motor = None


class Carriage:

    def __init__(self) -> None:
        self._ball_dropper = Gate()
        self._carriage_motor = CarriageMotor()
        self._home_position = 0
        self._cur_column = None


    def drop_ball_in_column_and_home(self, target_column: int) -> None:

        self.go_to_column(target_column)
        self.drop_ball()
        self.home()


    def recycle_current_ball(self) -> None:
        raise
        # TODO: define what column the recycle column is
        self.drop_ball_in_column_and_home(-1)


    def drop_ball(self) -> None:
        self._ball_dropper.open()
        sleep(0.5)
        self._ball_dropper.close()


    def go_to_column(self, target_column: int) -> None:
        
        distance_to_move = (target_column - self._cur_column) * INTER_COLUMN_DISTANCE

        self._carriage_motor.move(distance_to_move)
        # TODO: make sure this either blocks, or sleep some amount during drive

        self._cur_column = target_column

    
    def home(self) -> None:

        self.go_to_column(0)



class BallReader:

    def __init__(self) -> None:
        # Class for whatever method we use to get the value of the next ball
        pass

    def get_current_ball_color(self) -> str:
        ball_color = None
        # Must return either BLACK, WHITE, or None (if there's no ball at all there)
        return ball_color


class MarbleMirror:

    def __init__(self) -> None:

        # Main class for the whole mirror.

        self._board = MarbleBoard()
        self._elevator = Elevator()
        self._carriage = Carriage()
        self._board_dropper = Gate()
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