﻿import math


def euclidean(x1, x2):

    distance = math.sqrt((x2[1] - x1[1])*(x2[1] - x1[1]) + (x2[0] - x1[0])*(x2[0] - x1[0]))

    return distance


