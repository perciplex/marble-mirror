from re import I
from marble_mirror.components import MarbleBoard, BallState
import logging
from typing import Protocol


class DropPlanner(Protocol):
    board: MarbleBoard

    def select_drop_column(self) -> int:
        raise NotImplementedError


class ClosestValidColumn(DropPlanner):
    def __init__(self, board: MarbleBoard) -> None:
        self.board = board

    def select_drop_column(self, cur_ball_color: BallState, cur_cart_column: int) -> int:
        logging.debug(f"Looking for closest valid column for ball color: = {cur_ball_color}")
        frontier = self.board.get_frontier_at_depth(0)
        # valid_columns = [col for col in frontier if col == cur_ball_color]
        min_dist = 1000
        target_col = None
        for column in range(len(frontier)):
            logging.debug(f"Checking col {column}. {frontier[column]} vs {cur_ball_color}")
            if frontier[column] == cur_ball_color.value:
                cur_dist = abs(cur_cart_column - column)
                logging.debug(f"Current dist: {cur_dist}")
                if cur_dist < min_dist:
                    logging.debug(f"Col {column} was closest at {cur_dist}")
                    min_dist = cur_dist
                    target_col = column
        logging.info(f"Determined column {target_col} is best.")
        return target_col


class BalanceFrontier(DropPlanner):
    def get_valid_column_for_color_and_minimize_diff(self, ball_color: int, current_col: int):
        pass

        # TODO: Re-org this to function properly.
        # logging.debug(f"Looking for column for ball color: = {ball_color}")
        # valid_columns = [
        #     col for (col, queue) in enumerate(self._queues_list) if column_valid_for_color(queue, ball_color)
        # ]

        # # If we didn't find any, return None
        # if len(valid_columns) == 0:
        #     return None

        # frontier_balls = [queue[-1] for queue in self._queues_list if len(queue)]
        # n_white_balls = sum([b for b in frontier_balls if b == BallState.White.value])
        # n_black_balls = sum([b for b in frontier_balls if b == BallState.Black.value])

        # ball_most_needed = (
        #     ball_color
        #     if (n_black_balls == n_white_balls)
        #     else (BallState.Black.value if n_black_balls < n_white_balls else BallState.White.value)
        # )

        # # find the columns that have the ball we most want after the bottom one
        # frontier_improving_cols = []
        # for col in valid_columns:
        #     queue = self._queues_list[col]
        #     if len(queue) >= 2 and queue[-2] == ball_most_needed:
        #         frontier_improving_cols.append(col)

        # # if they exist, find the closest one
        # if frontier_improving_cols:
        #     target_col = min(frontier_improving_cols, key=lambda col: abs(col - current_col))
        # else:
        #     target_col = min(valid_columns, key=lambda col: abs(col - current_col))

        # self._queues_list[target_col].pop()
        # return target_col
