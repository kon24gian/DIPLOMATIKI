import math
import random
import sys
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('DARP')
from handleGeo.ConvCoords import ConvCoords
from nodesPlacementOptimization.Rotate import Rotate
from handleGeo import Dist
from handleGeo.real_world_parameter_parser import real_world

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
def initializeDARPGrid(randomInitPos, l, m, initialPos, megaNodes, theta, shiftX, shiftY, droneNo):
    DARPgrid = megaNodes[:, :, 2].astype(int)
    if randomInitPos:
        # Add drones in random initial positions
        c = 0
        while c < droneNo:
            # Initial positions of drones
            ind1 = random.randrange(0, l, 1)
            ind2 = random.randrange(0, m, 1)
            if DARPgrid[ind1][ind2] == 0:
                DARPgrid[ind1][ind2] = 2
                c += 1
    else:
        # Add drones in the closest mega-node
        i1 = 0
        i2 = 0

        # Convert initial positions from WGS84 to NED
        initPosNED = ConvCoords(cart1, cartObst1).convWGS84ToNED(initialPos)

        # Rotate and shift initial positions
        rotate = Rotate()
        rotate.setTheta(theta)
        initialPosNED = rotate.rotatePolygon(initPosNED)
        for i in range(len(initialPos)):
            initialPosNED[i][0] += shiftX
            initialPosNED[i][1] += shiftY

        for i in range(len(initialPosNED)):
            minDist = sys.float_info.max
            for j in range(l):
                for k in range(m):
                    distance = Dist.euclidean(initialPosNED[i], [megaNodes[j][k][0], megaNodes[j][k][1]])
                    if distance < minDist and megaNodes[j][k][2] == 0:
                        minDist = distance
                        i1 = j
                        i2 = k

            DARPgrid[i1][i2] = 2
            megaNodes[i1][i2][2] = -1
    # Change the value 2 to 0
    DARPgrid[DARPgrid == 2] = 0
    return DARPgrid


randomInitPos = True
count = 0

real_world_parameters = real_world()
print ("Geocoords :", real_world_parameters.geoCoords)
print ("Geoobstacles :", real_world_parameters.geoObstacles)
print ("Drone position:", real_world_parameters.initial_positions)


[cart1, cartObst1] = real_world_parameters.geo2cart()
print("Cart1:", cart1)
print("CartObst: ", cartObst1)
print("MegaNodes: ", real_world_parameters.megaNodes)


rows, cols, obstacles_positions = real_world_parameters.get_DARP_params()
print("Rows are:" , rows, "Cols are:", cols)


DARPgrid = initializeDARPGrid(randomInitPos, rows, cols, real_world_parameters.initial_positions,
                           real_world_parameters.megaNodes,
                           real_world_parameters.theta, real_world_parameters.shiftX,
                           real_world_parameters.shiftY,
                           real_world_parameters.droneNo)

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

obstacle_polygon = cartObst1[0] # Assuming there is only one obstacle polygon
vertices = obstacle_polygon.tolist()
print("Vertices of the obstacle polygon:")
for vertex in vertices:
    print(vertex)


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
def dist(p1, p2):
    return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5

def find_vertex_connecting_point(point, obstacle_polygon):
    for i in range(len(obstacle_polygon)):
        p1 = obstacle_polygon[i]
        p2 = obstacle_polygon[(i + 1) % len(obstacle_polygon)]
        if point_on_line_segment(point, p1, p2):
            if dist(point, p1) < dist(point, p2):
                return p1
            else:
                return p2
    return None

def point_on_line_segment(point, p1, p2):
    return min(p1[0], p2[0]) <= point[0] <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= point[1] <= max(p1[1], p2[1])


def find_vertices_on_perimeter(start_vertex, end_vertex, obstacle_polygon):
    start_index = obstacle_polygon.index(start_vertex)
    end_index = obstacle_polygon.index(end_vertex)

    # Clockwise path
    if start_index < end_index:
        vertices_cw = obstacle_polygon[start_index:end_index + 1]
    else:
        vertices_cw = obstacle_polygon[start_index:] + obstacle_polygon[:end_index + 1]

    # Counterclockwise path
    if start_index > end_index:
        vertices_ccw = obstacle_polygon[end_index:start_index + 1][::-1]
    else:
        vertices_ccw = obstacle_polygon[end_index:] + obstacle_polygon[:start_index + 1][::-1]

    return vertices_cw, vertices_ccw


def find_minimum_distance_path(pair, obstacle_polygon):
    start_point, end_point = pair[0], pair[1]

    # Find the vertices on the polygon that connect the start and end points
    start_vertex = find_vertex_connecting_point(start_point, obstacle_polygon)
    end_vertex = find_vertex_connecting_point(end_point, obstacle_polygon)

    # Create paths in both directions and calculate their lengths
    vertices_cw, vertices_ccw = find_vertices_on_perimeter(start_vertex, end_vertex, obstacle_polygon)

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

obstacle_polygon = cartObst1[0].tolist()




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

def find_insertion_points(boustrophedon_path, obstacle_path):
    start_x = obstacle_path[0][0][0]
    end_x = obstacle_path[-1][0][0]

    first_node, second_node = None, None
    for node in boustrophedon_path:
        if node[0] == start_x:
            first_node = node
        if node[0] == end_x:
            second_node = node

    return first_node, second_node

#  δεν ειναι ακομα ετοιμη αυτη η συναρτηση
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

        # Now insert the path into the updated path at the correct positions
        updated_path = updated_path[:start_insert_position] + path + updated_path[end_insert_position:]

    return updated_path


# Test the function

updated_path = insert_paths(matched_coords, all_paths)
print(updated_path)


import matplotlib.pyplot as plt


def plot_path(main_path, inserted_paths):
    # Unpack the main path into x and y coordinates for plotting
    x_coords, y_coords = zip(*main_path)

    # Create the plot
    plt.figure(figsize=(10, 10))
    plt.plot(x_coords, y_coords, '-o', color='blue')

    # Plot each inserted path in a different color
    colors = ['red', 'green', 'orange']  # Extend this list if you have more paths
    for path, color in zip(inserted_paths, colors):
        x_coords, y_coords = zip(*path)
        plt.plot(x_coords, y_coords, '-o', color=color)

    # Show the plot
    plt.show()

# Use the function
plot_path(updated_path, all_paths)



# Call the function to plot the DARP grid with nodes, path, intersection points, and obstacle vertices
plotDARPGridWithNodesAndPath(cart1, cartObst1, real_world_parameters.megaNodes, free_nodes_path, intersection_points, obstacle_polygon)



