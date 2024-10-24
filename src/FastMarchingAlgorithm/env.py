"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (0, 80)
        self.y_range = (0, 80)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            # [0, 0, 1, 30],
            # [0, 30, 50, 1],
            # [1, 0, 50, 1],
            # [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [22, 14, 8, 4], ## vat can 1
            [38, 20, 6, 10],
            [12, 30, 7, 5]
            # [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [54, 33, 5],
            [42, 42, 4],
            [18, 40, 4]
            # [37, 7, 3],
            # [37, 23, 3]

        ]
        return obs_cir
