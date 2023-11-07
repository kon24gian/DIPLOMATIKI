import matplotlib.patches as patches
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append('DARP')
from handleGeo.ConvCoords import ConvCoords
from handleGeo.real_world_parameter_parser import real_world
from itertools import chain
import json
from collections import Counter
from datetime import datetime

def plotDARPGridWithNodesAndPath(cart, cartObst, megaNodes, path, intersection_points, obstacle_polygon):
    # Plot DARP grid
    plt.figure()
    plt.plot(cart[:, 1], -cart[:, 0] + np.max(cart[:, 0]), '-k', label='DARP Grid')

    # Close the DARP grid perimeter line
    plt.plot([cart[0, 1], cart[-1, 1]], [-cart[0, 0] + np.max(cart[:, 0]), -cart[-1, 0] + np.max(cart[:, 0])], '-k')

    # Plot obstacles
    for obst in cartObst:
        plt.plot(obst[:, 1], -obst[:, 0] + np.max(cart[:, 0]), '-r', label='Obstacle')
        # Connect the first and last points to close the line
        plt.plot([obst[0, 1], obst[-1, 1]], [-obst[0, 0] + np.max(cart[:, 0]), -obst[-1, 0] + np.max(cart[:, 0])], '-r')

    # Plot mega nodes
    for i in range(len(megaNodes)):
        for j in range(len(megaNodes[i])):
            if megaNodes[i][j][2] == 1:
                plt.plot(megaNodes[i][j][1], -megaNodes[i][j][0] + np.max(cart[:, 0]), 'ro')  # Obstacle mega node
            else:
                plt.plot(megaNodes[i][j][1], -megaNodes[i][j][0] + np.max(cart[:, 0]), 'bo')  # Free mega node

    # Plot path
    if len(megaNodes) >= 1:
        path_x = [megaNodes[node[0]][node[1]][1] for node in path]
        path_y = [-megaNodes[node[0]][node[1]][0] + np.max(cart[:, 0]) for node in path]
        plt.plot(path_x, path_y, '-g', label='Path')

    # Plot intersection points
    if intersection_points:
        intersection_x = [point[1] for point in intersection_points]
        intersection_y = [-point[0] + np.max(cart[:, 0]) for point in intersection_points]
        plt.plot(intersection_x, intersection_y, 'o', color='purple', label='Intersection Points')

    # Plot vertices of the obstacle polygon
    if obstacle_polygon:
        vertex_x = [vertex[1] for vertex in obstacle_polygon]
        vertex_y = [-vertex[0] + np.max(cart[:, 0]) for vertex in obstacle_polygon]
        plt.plot(vertex_x, vertex_y, 'o', color='yellow', label='Obstacle Vertices')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('DARP Grid with Nodes and Path')
    plt.legend()
    plt.show()
def initializeDARPGrid(megaNodes):
    DARPgrid = megaNodes[:, :, 2].astype(int)

    return DARPgrid


randomInitPos = True
count = 0

real_world_parameters = real_world()
print("Geocoords :", real_world_parameters.geoCoords)
print("Geoobstacles :", real_world_parameters.geoObstacles)


[cart1, cartObst1] = real_world_parameters.geo2cart()
print("Cart1:", cart1)
print("CartObst: ", cartObst1)
print("MegaNodes: ", real_world_parameters.megaNodes)

rows, cols, obstacles_positions = real_world_parameters.get_DARP_params()
print("Rows are:" , rows, "Cols are:", cols)


DARPgrid = initializeDARPGrid(
                           real_world_parameters.megaNodes)


print( DARPgrid)

obstacles = [(i, j) for i in range(DARPgrid.shape[0]) for j in range(DARPgrid.shape[1]) if DARPgrid[i][j] == 1]

def boustrophedonTraversal(DARPgrid, obstacles):
    rows, cols = DARPgrid.shape
    path = []

    for i in range(rows):
        if i % 2 == 0:
            for j in range(cols):
                if DARPgrid[i][j] == 0 and (i, j) not in obstacles:
                    path.append((i, j))
        else:
            for j in range(cols - 1, -1, -1):
                if DARPgrid[i][j] == 0 and (i, j) not in obstacles:
                    path.append((i, j))


    return path


# Example usage
free_nodes_path = boustrophedonTraversal(DARPgrid,obstacles)
print("Free Nodes Path:")
print(free_nodes_path)

def get_matched_free_nodes_path(matched_free_nodes_path, megaNodes):
    matched_coords = []  # list to store matched coordinates
    for node in matched_free_nodes_path:
        if node is not None:
            i, j = node
            coord = tuple(megaNodes[i][j][:2])  # convert array to tuple
            print(f"({i},{j}) = {coord}")
            matched_coords.append(coord)  # append coordinate pair to list
    return matched_coords  # return list of coordinate pairs

# Call the function to get the matched free nodes path
matched_coords = get_matched_free_nodes_path(free_nodes_path, real_world_parameters.megaNodes)
print(matched_coords)





def plot_points_and_path(pair, path, obstacle_polygon):
    fig, ax = plt.subplots()

    # Plot obstacle polygon
    x, y = zip(*obstacle_polygon)
    ax.plot(x + (x[0],), y + (y[0],), 'k-', label='Obstacle Polygon')

    # Plot pair points
    x_pair, y_pair = zip(*pair)
    ax.scatter(x_pair, y_pair, color='red', label='Pair Points')

    # Plot path
    x_path, y_path = zip(*path)
    ax.plot(x_path, y_path, 'g-', label='Minimum Distance Path')

    ax.legend()
    plt.show()






wgs_coords = ConvCoords(real_world_parameters.geoCoords, real_world_parameters.geoObstacles).NEDToWGS84([matched_coords])
print("wgs_coords are: ", wgs_coords)






#JSON FILE
# Prepare the list of points as dictionaries




# Create an instance of the ConvCoords class with your geographic reference
conv_coords = ConvCoords(real_world_parameters.geoCoords, real_world_parameters.geoObstacles)
print("cart1",cart1)
# Convert the Cartesian coordinates (cart) to WGS-84 coordinates
cart1 = conv_coords.NEDToWGS841_a(cart1)


# Convert geocoords and geoobstacles to the desired format
area_polygon = [{"lat": lat[0][0], "lng": lat[0][1]} for lat in cart1]
print("area polygon", area_polygon)




# Flatten the list of lists into a single list of coordinate pairs
wgs_coords_flat = [coord for sublist in wgs_coords for coord in sublist]


# Aggregate them into a list under "polygons"
polygons = [
    {"points": area_polygon}


]
wgs_coords_f = wgs_coords_flat
# Create the desired format for paths

paths = [
    {"points": [{"lat": lat, "lng": lng} for lat, lng in wgs_coords_f]}
    ]

# Prepare the final JSON structure
json_data = {
    "sender": "Alert-driven_UAV_path_planning",
    "dateTime": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
    "horizontal_speed": 5,
    "paths": paths,
    "polygons": polygons,

}

# Convert dictionary to JSON string
json_string = json.dumps(json_data, indent=4)

# Optionally, write to a file
with open("visualize_path_data.json", "w") as f:
    f.write(json_string)


# Print JSON string
print(json_string)






# Flatten and make points unique (assumes your lists are similarly nested)



# Flatten wgs_coords and wgs_coords_r

flat_wgs_coords = [point for sublist in wgs_coords for point in sublist]

# Generate total_path_combined
total_path_combined =  flat_wgs_coords

# Count occurrences of each point
point_counter = Counter(map(tuple, total_path_combined))

# Debug counters
count_falses = 0
count_trues = 0

total_path_combined_with_flags = []

# Add flags to each point in total_path_combined
for point in total_path_combined:
    flag = True
    point_tuple = tuple(point)

    count_trues += 1
    total_path_combined_with_flags.append([*point, flag])

print(f"Total False Count: {count_falses}")
print(f"Total True Count: {count_trues}")


# Now total_path_combined_with_flags should contain the points with flags
print(total_path_combined_with_flags)



paths = []
path_id = 0  # Assuming a single path, you could loop over multiple paths and increment this

# Convert your coordinates into the desired format
waypoints = []
for lat, lng, photo in total_path_combined_with_flags:
    waypoints.append({
        "latitude": lat,
        "longitude": lng,
        "altitude": real_world_parameters.altitude,
        "heading": None,
        "photo": photo
    })

# Create a path dictionary for this pathID
path_dict = {
    "pathID": path_id,
    "photomode": "viewpoint",  # Replace with your actual data if available
    "waypoints": waypoints
}

paths.append(path_dict)

# Prepare the final JSON structure
json_data = {
    "sender": "Alert-driven_UAV_path_planning",
    "dateTime": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
    "horizontal_speed": 5,
    "paths": paths  # Add the paths
}

# Convert dictionary to JSON string
json_string = json.dumps(json_data, indent=4)

# Optionally, write to a file
with open("path_data.json", "w") as f:
    f.write(json_string)

# Print JSON string
print(json_string)
