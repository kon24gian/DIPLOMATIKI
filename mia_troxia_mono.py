import json
import matplotlib.pyplot as plt

# Load the JSON data from the file
with open('shots.geojson', 'r') as file:
    data = json.load(file)

# Extract X and Y coordinates from the translations
x_coords = [feature['properties']['translation'][0] for feature in data['features']]
y_coords = [feature['properties']['translation'][1] for feature in data['features']]

# Creating the 2D plot
plt.figure(figsize=(10, 6))

# Plotting the X and Y coordinates as dots
plt.scatter(x_coords, y_coords, marker='o', color='blue', label='Camera Shots')

# Plotting the path connecting the dots sequentially
plt.plot(x_coords, y_coords, linestyle='-', color='gray', label='Path')

# Adding labels and title
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Camera Shots and Sequential Path (2D)')

# Adding a legend
plt.legend()

# Showing the plot
plt.grid(True)
plt.show()
