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

    def setParameters(self, optimalTheta, range):
        self.optimalTheta = optimalTheta
        self.range = range

    def setTheta(self, theta):
        self.optimalTheta = theta

    def setRange(self, range):
        self.range = range

    def findOptimalTheta(self, scanDist, cartUnrotated, cartObstUnrotated, stepSize, shiftX, shiftY):
        testRange = int((self.range / stepSize))

        nodesIn = [0 for _ in range(testRange)]
        area = [0 for _ in range(testRange)]

        for i in range(testRange):
            cartObstRotated = [[None for i in range(len(cartObstUnrotated))]]

            for j in range(len(cartObstUnrotated)):
                cartObstRotated[j] = self.rotatePolygon(cartObstUnrotated[j])

            testTheta = NodesInPoly(self.rotatePolygon(cartUnrotated), cartObstRotated, scanDist, True, True, shiftX,
                                    shiftY)
            nodesIn[i] = testTheta.getMegaNodesInCount()
            area[i] = testTheta.getBoundingBoxArea()
            self.optimalTheta = self.optimalTheta + stepSize

        theta = float(MinMax.indMaxNodeMinArea(nodesIn, area) * stepSize)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" +
              "        Optimal Rotation Angle for Paths: ", theta, " degrees\n" +
              "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
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

    def rotateBackWaypoints(self, iWaypoints):

        minusTheta = -self.optimalTheta

        l = len(iWaypoints)
        waypoints = []

        for i in range(l):
            a = iWaypoints[i][0] * math.cos(math.radians(minusTheta)) - iWaypoints[i][1] * math.sin(
                math.radians(minusTheta))
            b = iWaypoints[i][0] * math.sin(math.radians(minusTheta)) + iWaypoints[i][1] * math.cos(
                math.radians(minusTheta))
            waypoints.append([a, b])

        return waypoints
