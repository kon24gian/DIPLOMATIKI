import math
import numpy as np
from handleGeo.NodesInPoly import NodesInPoly
from helpers import MinMax

from numba import njit

class Rotate:

    # def __init__(self):
    #
    # 	self.optimalTheta = 0
    # 	self.range = 90



    def setTheta(self, theta):
        self.optimalTheta = theta



    def rotatePolygon(self, cart):
        # Check if cart is a single point or a list of points
        if not isinstance(cart[0], (list, np.ndarray)):  # If cart is a single point
            x, y = cart
            x_rot = x * math.cos(math.radians(self.optimalTheta)) - y * math.sin(math.radians(self.optimalTheta))
            y_rot = x * math.sin(math.radians(self.optimalTheta)) + y * math.cos(math.radians(self.optimalTheta))
            return [x_rot, y_rot]

        else:  # If cart is a list of points
            l = len(cart)
            rotated = np.zeros((l, 2))  # Only two columns are needed for x and y
            for i in range(l):
                rotated[i][0] = cart[i][0] * math.cos(math.radians(self.optimalTheta)) - cart[i][1] * math.sin(
                    math.radians(self.optimalTheta))
                rotated[i][1] = cart[i][0] * math.sin(math.radians(self.optimalTheta)) + cart[i][1] * math.cos(
                    math.radians(self.optimalTheta))

            return rotated
