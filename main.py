import numpy as np
import laspy
import open3d as o3d
import time

import random
from random import randrange

from octree import Octree
from boundary_box import BoundaryBox
from graphic_utils import create_bounds_cube
from graphic_utils import create_vertical_line

#################### Constants

# subdivision cube relative point density in output screen.
# lower value = more density
# in open3d screen, tweak visual density with '-' and '=' buttons if necessary
# recommended values: 100 - 100000
cube_density = 100

# display in console how long it takes to process
# points when building tree
debug_build = False

# default center point for the octree 
# is average of min / max coordinates
center_offset = [0, 0, 0]

# max number of possible subdivisions
# mostly used values were (1-8)
subdivisions = 6

# processing the whole dataset is quite slow,
# so skipping points is possible for testing purposes
# set to 1 for full dataset
# set to n to only take every nth point
step = 200

########## Search constants

# Allowed movement directions during search
# 6: front, back, up, down, left, right
# 10: 6 + xy diagonals
# 26 (slow with higher subdivisions) : 10 + xz, yz, xyz diagonals
move_directions = [6, 10, 26]
move_dir_index = 0

# amount of random points to generate for searching
random_points = 15

# set to use custom chosen points for searching
# all paths should be found from these point pairs 
# (tested with 6 subdivisions and step 200)
use_preset_points = True

# display results in open3d screen
display_results = True

# display in console every path search result
# each path
debug_all_paths = True

# display in console how many paths were found (after every nth search)
# set to high value to skip
debug_nth_results = 10000
# debug_nth_results = 5


#############################################

#### Read dataset
point_data = []
with laspy.open('2743_1234.las') as fh:
    print('Points from Header:', fh.header.point_count)
    las = fh.read()
    point_data = np.stack([las.X, las.Y, las.Z], axis=0).transpose((1, 0))[::step]

######## Tree building ########
#### Find center point and cube length for root
minX = min(point_data[:, 0])
maxX = max(point_data[:, 0])
minY = min(point_data[:, 1])
maxY = max(point_data[:, 1])
minZ = min(point_data[:, 2])
maxZ = max(point_data[:, 2])

print((minX, maxX, minY, maxY, minZ, maxZ))

center_point = [((minX + maxX) / 2) + center_offset[0], ((minY + maxY) / 2) + center_offset[1], ((minZ + maxZ) / 2) + center_offset[2]]
x_length = maxX - minX
y_length = maxY - minY
z_length = maxZ - minZ
cube_dim_half = max([x_length, y_length, z_length]) / 2

#### Create and build tree
tree = Octree(BoundaryBox(center_point, cube_dim_half), [0],  point_data, layers_left=subdivisions)
print("Building octree.")
start = time.time()
tree.Build(debug=debug_build)
end = time.time()
print('Tree building time | {:.3f} s'.format(end - start))
###############################


###########A* Search###########
# find point pair from tree subdivided nodes
def node_and_point(node):
    if node == None:
        return (node, [])
    for child in node.children:
        (n, p) = node_and_point(child)
        if p != []:
            return (n, p)
    return (node, node.points)
(_, pointsA) = node_and_point(tree.children[1])
(_, pointsB) = node_and_point(tree.children[7])
pointA = pointsA[0]
pointB = pointsB[0]
# 

print("A* start")
paths = []
point_pairs = []
found = 0
all = 0

if use_preset_points:
    point_pairs.append([pointA, pointB])
    point_pairs.append([[ 2683,  95698, 194211], [ 71514,  98570, 221006]])
    point_pairs.append([[ 71767,  53425, 207503], [ 45605,  90434, 217796]])
    point_pairs.append([[ 59683,  80466, 211279], [ 98716,  88953, 241846]])
    point_pairs.append([[ 30451,  76719, 213223], [ 49794,  15292, 190978]])
for _ in range(random_points):
    point_pairs.append([point_data[randrange(len(point_data))], point_data[randrange(len(point_data))]])

for pointsAB in point_pairs:
    all_barriers = []

    start = time.time()
    (path, _) = tree.find_shortest_path(pointsAB[0], pointsAB[1], all_barriers=all_barriers, direction_count=move_directions[move_dir_index])
    end = time.time()

    if debug_all_paths:
        print('Path (found: {}) search time | {:.3f} s'.format(path != [], end - start))
    all = all + 1
    if path != []:
        paths.append(path)
        found = found + 1
    if all % debug_nth_results == 0:
        print("Paths found: {} / {}".format(found, all))

print("A* end, found {} / {} paths".format(found, all))

if not display_results:
    exit()

#### Display found paths (and all start / end points)

print("Creating display data.")
geometries = []
for path_nodes in paths:
    color_R = random.uniform(0, 1)
    color_G = random.uniform(0, 1)
    color_B = random.uniform(0, 1)
    for (node, _) in path_nodes:
        path_geom = create_bounds_cube(node.box, [color_R, color_G, color_B], cube_density)
        geometries.append(path_geom)
for pointsAB in point_pairs:
    color_value = random.uniform(0, 1)
    start_line = create_vertical_line(pointsAB[0], 5000, [0, color_value, 0])
    geometries.append(start_line)
    end_line = create_vertical_line(pointsAB[1], 5000, [0, 0, color_value])
    geometries.append(end_line)

geom = o3d.geometry.PointCloud()
geom.points = o3d.utility.Vector3dVector(point_data)
geometries.append(geom)

print("Displaying.")
o3d.visualization.draw_geometries(geometries)