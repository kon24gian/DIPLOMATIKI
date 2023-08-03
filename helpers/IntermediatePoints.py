""" ###############################################################################################"""
""" Class for calculating intermediate points on the calculated path """
import copy


class IntermediatePoints():
    def __init__(self, droneNo, WaypointsNED):
        self.droneNo = droneNo
        self.WaypointsNED = WaypointsNED
        self.nb_points = 1  # how many intermediate points


    """ ---------------------- Intermediate points --------------------- """
    def intermediates(self, p1, p2, nb_points=8):
        """"Return a list of nb_points equally spaced points
        between p1 and p2"""
        # If we have e.g. 8 intermediate points, we have 8+1=9 spaces between p1 and p2
        x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
        y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

        return [[p1[0] + i * x_spacing, p1[1] + i * y_spacing]
                for i in range(1, nb_points + 1)]

    def WaypointsNED_UPDATE(self):


        WaypointsNED_copy = copy.deepcopy(self.WaypointsNED)

        for i in range(self.droneNo):
            for j in range(len(WaypointsNED_copy[i][0]) - 1):
                intermediateWP = self.intermediates([WaypointsNED_copy[i][0][j][0], WaypointsNED_copy[i][0][j][1]],
                                               [WaypointsNED_copy[i][0][j + 1][0],
                                                WaypointsNED_copy[i][0][j + 1][1]], self.nb_points)

                for n in range(self.nb_points):
                    self.WaypointsNED[i][0].insert((self.nb_points + 1) * j + n + 1, intermediateWP[n])

        return self.WaypointsNED


