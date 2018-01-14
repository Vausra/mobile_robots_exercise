import numpy as np
import random


class Particle:

    def __init__(self, position_x, position_y, omega):
        assert omega <= np.pi, 'only values between 0 and pi are allowed'
        assert omega >= 0, 'only values between 0 and pi are allowed'

        # protected variables
        self._position_x = position_x
        self._position_y = position_y
        self._omega = omega


    def get_pose(self):
        return self._position

    def get_omega(self):
        return self._omega