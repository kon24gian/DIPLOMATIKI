from helpers import MinMax
import numpy as np
from numba import njit
from handleGeo.InPolygon import is_inside_polygon

@njit
def count_non_zero(np_arr):
    return len(np.nonzero(np_arr)[0])

class NodesInPoly(object):

        def __init__(self, cartCoords, cartObst, scanDist, captDist, hideInfo):
            self.scanDist = scanDist
            self.captDist = captDist
            self.hideInfo = hideInfo
            if len(cartObst) == 0:
                self.cartObst = np.zeros((1, 4, 2))
            else:
                self.cartObst = np.array(cartObst)

            self.x_nodeDistance = scanDist
            self.y_nodeDistance = captDist

            self.nodeIntervalOffset = self.x_nodeDistance / 4  # Half the distance
            self.polygonCoordinates = cartCoords
            self.xMax = MinMax.xMax(self.polygonCoordinates) + self.x_nodeDistance
            self.xMin = MinMax.xMin(self.polygonCoordinates) - self.x_nodeDistance
            self.yMax = MinMax.yMax(self.polygonCoordinates) + self.y_nodeDistance
            self.yMin = MinMax.yMin(self.polygonCoordinates) - self.y_nodeDistance

            self.xRngMeters = self.xMax - self.xMin
            self.yRngMeters = self.yMax - self.yMin

            self.xNodes = int((self.xRngMeters / self.x_nodeDistance))
            self.yNodes = int((self.yRngMeters / self.y_nodeDistance))

            if not hideInfo:
                print("Bounding box: ", self.xRngMeters, " x ", self.yRngMeters, " meters")
                print("xNodes: ", self.xNodes)
                print("yNodes: ", self.yNodes)
                print("Total number of mega-nodes: ", self.xNodes * self.yNodes)

            self.betterCoverage(self.xMin, self.yMin, self.x_nodeDistance, self.y_nodeDistance, hideInfo)

        def betterCoverage(self, xMin, yMin, x_nodeDistance, y_nodeDistance, hideInfo):
            self.megaNodesCount = 0
            self.megaNodes = np.zeros((self.xNodes, self.yNodes, 3))
            for i in range(self.xNodes):
                for j in range(self.yNodes):
                    self.megaNodes[i][j][0] = xMin + i * x_nodeDistance
                    self.megaNodes[i][j][1] = yMin + j * y_nodeDistance

                    if is_inside_polygon([self.megaNodes[i][j][0], self.megaNodes[i][j][1]], self.polygonCoordinates):
                        self.megaNodes[i][j][2] = 0
                        self.megaNodesCount += 1
                    else:
                        self.megaNodes[i][j][2] = 1

            if not hideInfo:
                print("Number of mega-nodes inside polygon: " + str(self.megaNodesCount))
                print("Number of sub-nodes that will be used for trajectories: " + str(4.0 * self.megaNodesCount))

            self.checkInPolyAndObstacle(self.megaNodes, self.cartObst)

        @staticmethod
        @njit()
        def checkInPolyAndObstacle(megaNodes, cartObst):
            for i in range(len(megaNodes)):
                for j in range(len(megaNodes[0])):
                    if megaNodes[i][j][2] != 1:
                        for k in range(len(cartObst)):
                            if is_inside_polygon([megaNodes[i][j][0], megaNodes[i][j][1]], cartObst[k]):
                                megaNodes[i][j][2] = 1
                                break

        def getPolygonArea(self):
            area = 0
            numPoints = len(self.polygonCoordinates)
            j = numPoints - 1  # The last vertex is the 'previous' one to the first
            for i in range(numPoints):
                area = area + abs((self.polygonCoordinates[j][0] + self.polygonCoordinates[i][0]) * (
                        self.polygonCoordinates[j][1] - self.polygonCoordinates[i][1]))
                j = i  # j is previous vertex to i

            return area / 2


        def getMegaNodesInCount(self):
            return self.megaNodesCount

        def getMegaNodes(self):
            return self.megaNodes



        def getBoundingBoxArea(self):
            return self.xRngMeters * self.yRngMeters

        def getMegaNodesInCount(self):
            return self.megaNodesCount

        def getMegaNodes(self):
            return self.megaNodes

