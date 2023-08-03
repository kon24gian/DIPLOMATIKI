
import random
import sys
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('DARP')
from handleGeo.ConvCoords import ConvCoords
from nodesPlacementOptimization.Rotate import Rotate
from handleGeo import Dist
from handleGeo.real_world_parameter_parser import real_world

def insert_path(first_path, second_path):
    # Get the first and last points of the first path
    first_point = first_path[0]
    last_point = first_path[-1]

    # Initialize start and end insertion indices
    start_insert_position = None
    end_insert_position = None

    # Loop over the second path to find the matching points
    for i in range(len(second_path) - 1):
        if second_path[i][1] < first_point[1] and second_path[i+1][1] > last_point[1]:
            start_insert_position = i+1
            end_insert_position = i+2
            break

    # Check if we found a match
    if start_insert_position is None or end_insert_position is None:
        print("No appropriate position found for insertion.")
        return second_path

    # Now insert the first path into the second path at the correct positions
    return second_path[:start_insert_position] + first_path + second_path[end_insert_position:]

# Test the function
first_path = [(294.6305831863359, 410.6370657396698), [223.22752023745204, 416.88916346615906], [244.32238300148717, 313.29762545677363], (294.6305831863359, 307.3341991105616)]
second_path = [(254.63058318633594, 64.95910148402251), (294.6305831863359, 64.95910148402251), (294.6305831863359, 104.95910148402251), (294.6305831863359, 144.9591014840225), (294.6305831863359, 184.9591014840225), (294.6305831863359, 224.9591014840225), (294.6305831863359, 264.95910148402254), (294.6305831863359, 424.95910148402254), (294.6305831863359, 464.95910148402254), (294.6305831863359, 504.95910148402254), (294.6305831863359, 544.9591014840225), (294.6305831863359, 584.9591014840225), (294.6305831863359, 624.9591014840225), (294.6305831863359, 664.9591014840225), (294.6305831863359, 704.9591014840225), (334.6305831863359, 664.9591014840225), (334.6305831863359, 624.9591014840225)]
inserted_path = insert_path(first_path, second_path)
print(inserted_path)

