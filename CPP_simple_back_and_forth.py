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


obstacle_polygon = cartObst1 # Assuming there is only one obstacle polygon

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

def extract_x_coordinates(megaNodes):
    x_coordinates = set()
    for row in megaNodes:
        for node in row:
            x_coordinates.add(node[0])
    return list(x_coordinates)

grid_lines = extract_x_coordinates(real_world_parameters.megaNodes)
print("Grid Lines:")
print(grid_lines)


def find_intersection_points(lines, obstacle_polygon):
    intersection_points = []
    for line in lines:
        x = line  # x-coordinate of the line

        for i in range(len(obstacle_polygon)):
            p1 = obstacle_polygon[i]
            p2 = obstacle_polygon[(i + 1) % len(obstacle_polygon)]
            x1, y1 = p1  # x, y coordinates of the first vertex of the edge
            x2, y2 = p2  # x, y coordinates of the second vertex of the edge

            if x1 == x2:
                continue  # Skip vertical edges

            # Check if the line intersects with the edge
            if (x1 <= x <= x2) or (x2 <= x <= x1):
                intersect_y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                intersection_points.append((x, intersect_y))

    return intersection_points

intersection_points = find_intersection_points(grid_lines, obstacle_polygon)
print("Intersection Points:")
for point in intersection_points:
    print(point)
def path_length(path):
    length = 0.0
    if len(path) < 2:
        return length
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        length += ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return length
def insert_intersection(polygon, p1, p2, intersection):
    index = polygon.index(p1)
    next_vertex = polygon[(index + 1) % len(polygon)]
    if next_vertex == p2:
        polygon.insert(index + 1, intersection)
    else:
        polygon.insert(index, intersection)
    return polygon

def point_on_line_segment(point, p1, p2):
    return min(p1[0], p2[0]) <= point[0] <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= point[1] <= max(p1[1], p2[1])


def get_edge_from_intersection(point, obstacle_polygon):
    """Returns the edge (as a tuple of two vertices) of the obstacle_polygon
    that is intersected by the point."""
    min_distance = float('inf')
    closest_edge = None

    for i in range(len(obstacle_polygon)):
        p1 = obstacle_polygon[i]
        p2 = obstacle_polygon[(i + 1) % len(obstacle_polygon)]

        # Calculate the distance from the intersection point to the edge
        distance = distance_point_to_segment(point, p1, p2)

        if distance < min_distance:
            min_distance = distance
            closest_edge = (p1, p2)

    if closest_edge:
        print("Intersection Point:", point)
        print("Start Vertex of the Edge:", closest_edge[0])
        print("End Vertex of the Edge:", closest_edge[1])
        return closest_edge

    return None


# Define a function to calculate the distance from a point to a line segment
def distance_point_to_segment(point, p1, p2):
    x, y = point
    x1, y1 = p1
    x2, y2 = p2

    A = x - x1
    B = y - y1
    C = x2 - x1
    D = y2 - y1

    dot = A * C + B * D
    len_sq = C * C + D * D
    param = -1

    if len_sq != 0:  # in case of a zero length line
        param = dot / len_sq

    if param < 0:
        x = x1
        y = y1
    elif param > 1:
        x = x2
        y = y2
    else:
        x = x1 + param * C
        y = y1 + param * D

    dx = x - point[0]
    dy = y - point[1]
    return (dx * dx + dy * dy) ** 0.5

def find_vertices_on_perimeter(start_vertex, end_vertex, obstacle_polygon):
    vertices_cw = []
    vertices_ccw = []

    # Start the traversal at start_vertex
    index = obstacle_polygon.index(start_vertex)

    # Traverse clockwise until we reach end_vertex
    while obstacle_polygon[index] != end_vertex:
        vertices_cw.append(obstacle_polygon[index])
        index = (index + 1) % len(obstacle_polygon)
    vertices_cw.append(end_vertex)

    # Reset the index for counterclockwise traversal
    index = obstacle_polygon.index(start_vertex)

    # Traverse counterclockwise until we reach end_vertex
    while obstacle_polygon[index] != end_vertex:
        vertices_ccw.append(obstacle_polygon[index])
        index = (index - 1) % len(obstacle_polygon)
    vertices_ccw.append(end_vertex)

    return vertices_cw, vertices_ccw

def find_minimum_distance_path(pair, obstacle_polygon):
    start_point, end_point = pair[0], pair[1]

    # Assuming you have a function that gives you the edge for each intersection
    start_edge = get_edge_from_intersection(start_point, obstacle_polygon)
    end_edge = get_edge_from_intersection(end_point, obstacle_polygon)

    # Insert the intersection points into our polygon vertices list
    new_polygon = list(obstacle_polygon)  # Create a copy
    insert_intersection(new_polygon, start_edge[0], start_edge[1], start_point)
    insert_intersection(new_polygon, end_edge[0], end_edge[1], end_point)

    # Create paths in both directions and calculate their lengths
    vertices_cw, vertices_ccw = find_vertices_on_perimeter(start_point, end_point, new_polygon)
    vertices_cw = [point for point in vertices_cw if point != start_point and point != end_point]
    vertices_ccw = [point for point in vertices_ccw if point != start_point and point != end_point]

    path1 = [start_point] + vertices_cw + [end_point]
    length1 = path_length(path1)

    path2 = [start_point] + vertices_ccw + [end_point]
    length2 = path_length(path2)

    # Return the shortest path
    return path1 if length1 < length2 else path2
def print_path_info(pair, path):
    start_point = pair[0]
    end_point = pair[1]

    print(f"Start Point: {start_point}")
    print(f"End Point: {end_point}")
    print("Vertices:")
    for vertex in path[1:-1]:
        print(vertex)
    print()

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

obstacle_polygon = cartObst1
# Create a list to store all paths of intersection points
all_paths = []

# Iterate through each pair of points and find the shortest path
for i in range(0, len(intersection_points), 2):
    pair = intersection_points[i:i + 2]
    path = find_minimum_distance_path(pair, obstacle_polygon)
    all_paths.append(path)  # Append the path to the list of all_paths
    print_path_info(pair, path)
    plot_points_and_path(pair, path, obstacle_polygon)


print(all_paths)
def insert_paths(main_path, paths_to_insert):
    # Copy the main path to keep the original data
    updated_path = main_path.copy()

    for path in paths_to_insert:
        # Get the first and last points of the path
        first_point = path[0]
        last_point = path[-1]

        # Initialize start and end insertion indices
        start_insert_position = None
        end_insert_position = None

        # Loop over the updated path to find the matching points
        for i in range(len(updated_path) - 1):
            if (updated_path[i][1] <= first_point[1] < updated_path[i+1][1] and updated_path[i+1][1] > last_point[1] >= updated_path[i][1]) or (updated_path[i][1] >= first_point[1] > updated_path[i+1][1] and updated_path[i+1][1] < last_point[1] <= updated_path[i][1]):
                if updated_path[i][0] == first_point[0] and updated_path[i+1][0] == last_point[0]:
                    start_insert_position = i+1
                    end_insert_position = i+2
                    break

        # Check if we found a match
        if start_insert_position is None or end_insert_position is None:
            print(f"No appropriate position found for insertion of path: {path}")
            continue

        # Determine if the path should be reversed
        if (updated_path[start_insert_position - 1][1] > updated_path[end_insert_position - 1][1] and first_point[1] < last_point[1]) or (updated_path[start_insert_position - 1][1] < updated_path[end_insert_position - 1][1] and first_point[1] > last_point[1]):
            path = path[::-1]

        # Now insert the path into the updated path at the correct positions
        for i, point in enumerate(path):
            updated_path.insert(start_insert_position + i, point)

    return updated_path

# Test the function
updated_path = insert_paths(matched_coords, all_paths)

print(updated_path)

def plot_path(main_path, inserted_paths, polygon_coords, obstacle_coords):
    # Unpack the main path into x and y coordinates for plotting
    x_coords, y_coords = zip(*main_path)

    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot the main path
    ax.plot(x_coords, y_coords, '-o', color='blue')

    # Plot each inserted path in a different color
    colors = ['red', 'green', 'orange', 'purple', 'black', 'pink']  # Extend this list if you have more paths
    for path, color in zip(inserted_paths, colors):
        x_coords, y_coords = zip(*path)
        ax.plot(x_coords, y_coords, '-o', color=color)

    # Create a polygon for the outer boundary and add it to the plot
    poly = patches.Polygon(polygon_coords, fill=None, edgecolor='purple')
    ax.add_patch(poly)



    # Show the plot
    plt.show()

# Use the function
plot_path(updated_path, all_paths, cart1, cartObst1)

# Call the function to plot the DARP grid with nodes, path, intersection points, and obstacle vertices
plotDARPGridWithNodesAndPath(cart1, cartObst1, real_world_parameters.megaNodes, free_nodes_path, intersection_points, obstacle_polygon)

wgs_coords = ConvCoords(real_world_parameters.geoCoords, real_world_parameters.geoObstacles).NEDToWGS84([updated_path])
print("wgs_coords are: ", wgs_coords)



# Flatten the list and convert inner lists to tuples
all_paths = [tuple(point) if isinstance(point, list) else point for sublist in all_paths for point in sublist]

wgs_coords_obst_paths = ConvCoords(real_world_parameters.geoCoords, real_world_parameters.geoObstacles).NEDToWGS84([all_paths])


#JSON FILE
# Prepare the list of points as dictionaries



obstacle_polygon = list(chain.from_iterable(real_world_parameters.geoObstacles))


# Create an instance of the ConvCoords class with your geographic reference
conv_coords = ConvCoords(real_world_parameters.geoCoords, real_world_parameters.geoObstacles)
print("cart1",cart1)
# Convert the Cartesian coordinates (cart) to WGS-84 coordinates
cart1 = conv_coords.NEDToWGS841_a(cart1)


# Convert geocoords and geoobstacles to the desired format
area_polygon = [{"lat": lat[0][0], "lng": lat[0][1]} for lat in cart1]
print("area polygon", area_polygon)

obstacle_polygon = conv_coords.NEDToWGS841_a(obstacle_polygon)

obstacle_polygon = [{"lat": lat[0][0], "lng": lat[0][1]} for lat in obstacle_polygon]
print("obstacle_polygon", obstacle_polygon)


# Flatten the list of lists into a single list of coordinate pairs
wgs_coords_flat = [coord for sublist in wgs_coords for coord in sublist]


# Aggregate them into a list under "polygons"
polygons = [
    {"points": area_polygon}

]
obstacles = [
    {"points": obstacle_polygon}
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
    "obstacles": obstacles
}

# Convert dictionary to JSON string
json_string = json.dumps(json_data, indent=4)

# Optionally, write to a file
with open("visualize_path_data.json", "w") as f:
    f.write(json_string)


# Print JSON string
print(json_string)






# Flatten and make points unique (assumes your lists are similarly nested)

flat_wgs_coords_obst_paths = {tuple(point) for sublist in wgs_coords_obst_paths for point in sublist}

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
    if point_tuple in flat_wgs_coords_obst_paths:
        print(f"Found in flat_wgs_coords_obst_paths: {point}, Count: {point_counter[point_tuple]}")
        flag = False
        count_falses += 1
    else:
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
