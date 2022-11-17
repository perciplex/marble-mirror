import logging

from marble_mirror.components import MarbleBoard, BallState
from typing import Protocol


class DropPlanner(Protocol):
    board: MarbleBoard

    def get_drop_column(self, cur_ball_color: BallState, cur_cart_column: int) -> int:
        raise NotImplementedError


class ClosestValidColumn(DropPlanner):
    """
    Planner which chooses the closest valid column from the cart position.
    """

    def __init__(self, board: MarbleBoard) -> None:
        self.board = board

    def get_drop_column(self, cur_ball_color: BallState, cur_cart_column: int) -> int:
        target_col = None

        logging.debug(f"Looking for closest valid column for ball color: = {cur_ball_color}")
        frontier = self.board.get_frontier_at_depth(0)

        candidates = [column for (column, color) in enumerate(frontier) if color == cur_ball_color]
        if candidates:
            target_col = min(candidates, key=lambda target: abs(target - cur_cart_column))
        logging.info(f"Determined column {target_col} is best.")
        return target_col


class BalanceFrontier(DropPlanner):
    """
    Planner which attempts to balance the current and second frontier needed balls.
    Will look at the next two frontiers, determine which color is needed most to balance what is needed,
    then find the candidate columns that needs the current ball color and balancers frontier.

    Chooses the closet candidate column to the current cart position.
    """

    def __init__(self, board: MarbleBoard) -> None:
        self.board = board

    def get_drop_column(self, cur_ball_color: BallState, cur_cart_column: int) -> int:

        frontier = self.board.get_frontier_at_depth(0)
        candidates = [column for (column, color) in enumerate(frontier) if color == cur_ball_color]
        if not candidates:
            return None

        frontier1 = self.board.get_frontier_at_depth(1)
        tot_white_in_fontiers = len([color for color in (frontier + frontier1) if color == BallState.White])
        tot_black_in_fontiers = len([color for color in (frontier + frontier1) if color == BallState.Black])

        if tot_black_in_fontiers > tot_white_in_fontiers:
            needed_color = BallState.White
        elif tot_white_in_fontiers > tot_black_in_fontiers:
            needed_color = BallState.Black
        else:
            needed_color = cur_ball_color

        # Check if we have any candidate columns that improve the frontier color balance
        imp_candidates = [column for column in candidates if frontier1[column] == needed_color]
        if imp_candidates:
            # If yes, overwrite the eligible candidate list
            candidates = imp_candidates

        # Choose the closest of the final candidates list.
        target_col = min(candidates, key=lambda target: abs(target - cur_cart_column))
        logging.info(f"Determined column {target_col} is best.")
        return target_col


if __name__ == "__main__":
    logging.basicConfig(format="%(levelname)s:%(message)s", level=logging.DEBUG)
    board = MarbleBoard(n_cols=4, n_rows=4)

    image = [[0, 1, 1, 0]]
    board.set_new_board(image)
    planner = ClosestValidColumn(board=board)
    print(planner.get_drop_column(BallState.Black, 2))
