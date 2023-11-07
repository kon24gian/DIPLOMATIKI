
import json
import math
import time
import numpy as np
from handleGeo.ConvCoords import ConvCoords
from handleGeo.NodesInPoly import NodesInPoly
from nodesPlacementOptimization.Rotate import Rotate
class real_world:
    def __init__(self):
        """ DEFINE THE DRONE'S SPECS """
        # # Phantom 4 pro Image Sensor specs
        self.HFOV = 73.4
        self.hRes = 5472
        self.ImageWidth = 5472
        self.ImageHeight = 3648
        self.SensorWidth = 13.2
        self.SensorHeight = 8
        self.FocalLength = 8.8
        #self.VFOV = 2 * math.atan(self.SensorHeight / (2 * self.FocalLength))
        self.VFOV = 45.7
        # RedEdge-M Image Sensor specs
        # HFOV = 46
        # hRes = 1280
        # ImageWidth = 1280
        # ImageHeight = 960
        # SensorWidth = 13.2
        # SensorHeight = 8
        # FocalLength = 5.4

        self.real_world_parameters()

    def real_world_parameters(self):
        file = open('orth.json')
        data = json.load(file)

        self.geoCoords = []
        self.geoObstacles = []

        for i in data['polygon']:
            self.geoCoords.append([i.get("lat"), i.get("long")])

        if len(data['obstacles']) > 0:
            self.geoObstacles = [[] for _ in range(len(data['obstacles']))]

            for i in range(len(data['obstacles'])):
                for j in data['obstacles'][i]:
                    self.geoObstacles[i].append([j.get("lat"), j.get("long")])

        self.altitude = data['altitude']
        self.sidelap = data['sidelap']
        self.frontlap = data['frontlap']


        self.scanDist = float("{:.2f}".format(self.covered() * (1 - self.sidelap / 100)))
        print("Scanning Distance:", self.scanDist)

        self.captDist = float("{:.2f}".format(self.covered_w() * (1 - self.frontlap / 100)))
        print("Capturing Distance:", self.captDist)



    def geo2cart(self):
        # Convert geographical to local cartesian coordinates
        self.NED_Coords = ConvCoords(self.geoCoords, self.geoObstacles).polygonWGS84ToNED()

        if len(self.geoObstacles) > 0:
            self.obstNED = ConvCoords(self.geoCoords, self.geoObstacles).obstaclesToNED()
        else:
            self.obstNED = []  # [np.complex64(x) for x in range(0)]


        self.rotate = Rotate()
        self.theta =45
        self.rotate.setTheta(self.theta)
        cart = self.rotate.rotatePolygon(self.NED_Coords)


        cartObst = [[] for _ in range(len(self.obstNED))]


        for i in range(len(self.obstNED)):
            cartObst[i] = self.rotate.rotatePolygon(self.obstNED[i])


        print(f"-  theta: {self.theta}")
        # Build grid for paths
        # Build grid for paths
        NodesInPoly_var = NodesInPoly(cart, cartObst, scanDist=self.scanDist, captDist=self.captDist,
                                      hideInfo=False,)

        megaNodesIn = NodesInPoly_var.getMegaNodesInCount()
        self.megaNodes = NodesInPoly_var.getMegaNodes()



        print(f"User's defined polygon area: {NodesInPoly_var.getPolygonArea()} square meters")
        return [cart , cartObst]
    def geo2cart2(self):
        # Convert geographical to local cartesian coordinates
        self.NED_Coords = ConvCoords(self.geoCoords, self.geoObstacles).polygonWGS84ToNED()

        if len(self.geoObstacles) > 0:
            self.obstNED = ConvCoords(self.geoCoords, self.geoObstacles).obstaclesToNED()
        else:
            self.obstNED = []  # [np.complex64(x) for x in range(0)]

        self.rotate = Rotate()
        self.theta = 135
        self.rotate.setTheta(self.theta)
        cart = self.rotate.rotatePolygon(self.NED_Coords)

        cartObst = [[] for _ in range(len(self.obstNED))]

        for i in range(len(self.obstNED)):
            cartObst[i] = self.rotate.rotatePolygon(self.obstNED[i])


        print(f"-  theta: {self.theta}")


        # Build grid for paths
        NodesInPoly_var = NodesInPoly(cart, cartObst, scanDist=self.scanDist,captDist=self.captDist,
                                      hideInfo=False)

        megaNodesIn = NodesInPoly_var.getMegaNodesInCount()
        self.megaNodes = NodesInPoly_var.getMegaNodes()


        print(f"User's defined polygon area: {NodesInPoly_var.getPolygonArea()} square meters")
        return [cart , cartObst]

    def get_DARP_params(self):
        # DARP parameters
        rows = len(self.megaNodes)
        cols = len(self.megaNodes[0])

        """ In DARPgrid 0 stands for free space 
            1 stands for Obstacle
            2 stands for Drone
        """
        DARPgrid = self.megaNodes[:, :, 2].astype(int)

        # """ ADDON for DARP_initial_positions module """
        grid_positions = np.arange(0, rows * cols).reshape(rows, cols)
        obs_pos_to_grid = np.where(DARPgrid == 1)
        obs_pos_to_grid = np.asarray(obs_pos_to_grid).T
        obs_pos = []

        for elem in obs_pos_to_grid:
            obs_pos.append(grid_positions[elem[0], elem[1]])

        return rows, cols, obs_pos



    def getTanFromDegrees(self, degrees):
        return math.tan(degrees * math.pi / 180)

    #regarding dh
    def covered(self):
        return 2 * self.altitude * self.getTanFromDegrees(self.HFOV / 2)

    #regarding dv
    def covered_w(self):
        return 2 * self.altitude * self.getTanFromDegrees(self.VFOV / 2)
    def GSDh(self):
        return ((self.altitude * 100) * (self.SensorHeight / 10)) / ((self.FocalLength / 10) * self.ImageHeight)

    def GSDw(self):
        return ((self.altitude * 100) * (self.SensorWidth / 10)) / ((self.FocalLength / 10) * self.ImageWidth)

