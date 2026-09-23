import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# Your raw data string
raw_data = """
0.5876529923305989,2.671486606795326,4,0.15
-0.41503583525462845,3.6006352232706047,3,0.15
1.0514780219290134,1.9160308589482737,7,0.15
-1.0890840506727018,2.301676077488529,2,0.15
"""

x_coords = []
y_coords = []
labels = []

# Parse the data to extract the 1st, 2nd, and 3rd values
for line in raw_data.strip().split('\n'):
    parts = line.split(',')
    if len(parts) >= 3:
        x_coords.append(float(parts[0].strip()))
        y_coords.append(float(parts[1].strip()))
        labels.append(parts[2].strip())  # Store the 3rd value

# Create the plot
fig, ax = plt.subplots(figsize=(8, 8))

# Center circle at 0,0 with radius 22.5 cm (0.225 m)
center_radius = 0.225
center_circle = plt.Circle((0, 0), center_radius, color='red', alpha=0.4)
ax.add_patch(center_circle)

# Data points: 15 cm diameter (0.075 m radius)
marker_radius = 0.15 

# Add points and layer the text over them
for x, y, label in zip(x_coords, y_coords, labels):
    # Draw the marker
    circle = plt.Circle((x, y), marker_radius, color='blue', alpha=0.7)
    ax.add_patch(circle)
    
    # Write the 3rd value onto the center of the marker
    ax.text(x, y, label, color='white', fontweight='bold', ha='center', va='center')

# Calculate the furthest point to create symmetrical limits
if x_coords and y_coords:
    max_val = max(max([abs(x) for x in x_coords]), max([abs(y) for y in y_coords]), center_radius)
else:
    max_val = center_radius

# Add 20% padding + the marker radius
limit = (max_val * 1.2) + marker_radius 

# Set symmetrical limits to perfectly center 0,0
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)

# Force the X and Y axes to have the exact same scale
ax.set_aspect('equal')

# Draw distinct crosshairs at 0,0
ax.axhline(0, color='black', linewidth=1.2)
ax.axvline(0, color='black', linewidth=1.2)

# Formatting
plt.title('Coordinate System (Centered at 0,0)')
plt.xlabel('Meters (X)')
plt.ylabel('Meters (Y)')
plt.grid(True, linestyle='--', alpha=0.5)

# Add a custom legend
custom_lines = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor='red', alpha=0.4, markersize=10),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', alpha=0.7, markersize=10)
]
ax.legend(custom_lines, ['Center (r=22.5cm)', 'Data Points (d=15cm)'])

# Display the plot
plt.show()