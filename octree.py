import numpy as np
import open3d as o3d
import time
from random import randrange
from graphic_utils import create_vertical_line
from graphic_utils import create_bounds_cube
import random
import math
import heapq
from boundary_box import BoundaryBox

########### util functions #############
global_directions = []

# helper function to get cube directions.
# use of global variable is not necessary,
# but slightly increases performance due to avoiding repeated initializations
def get_directions(layer, new_array_distance=None, direction_count=6):
    global global_directions
    if len(global_directions) < layer - 1:
        if new_array_distance == None:
            print("---------------warning: returning empty directions---------------")
            return []
        else:
            new_directions = None
            if direction_count == 6:
                new_directions = make_6_directions(new_array_distance)
            elif direction_count == 10:
                new_directions = make_10_directions(new_array_distance)
            else:
                new_directions = make_26_directions(new_array_distance)
            global_directions.append(new_directions)
    return global_directions[layer - 2]

# make_directions functions are explained in main -> Constants

def make_6_directions(distance):
    return (
        [
            [0, 0, distance], [0, 0, -distance], [0, distance, 0], 
            [0, -distance, 0], [distance, 0, 0], [-distance, 0, 0]
        ], 
        [
            distance, distance, distance, distance, distance, distance
        ])

def make_10_directions(distance):
    dist2 = distance * 1.414
    dist3 = distance * 1.732
    return (
        [
            [0, 0, distance], [0, 0, -distance], [0, distance, 0], 
            [0, -distance, 0], [distance, 0, 0], [-distance, 0, 0],

            [distance, distance, 0],
            [-distance, distance, 0],
            [distance, -distance, 0],
            [-distance, -distance, 0],
        ], 
        [
            distance, distance, distance, distance, distance, distance, 
            dist2, dist2, dist2, dist2
        ])

def make_26_directions(distance):
    dist2 = distance * 1.414
    dist3 = distance * 1.732
    return (
        [
            [0, 0, distance], [0, 0, -distance], [0, distance, 0], 
            [0, -distance, 0], [distance, 0, 0], [-distance, 0, 0],

            [0, distance, distance], [distance, 0, distance], [distance, distance, 0],
            [0, -distance, distance], [-distance, 0, distance], [-distance, distance, 0],
            [0, distance, -distance], [distance, 0, -distance], [distance, -distance, 0],
            [0, -distance, -distance], [-distance, 0, -distance], [-distance, -distance, 0],

            [distance, distance, distance], [distance, distance, -distance], [distance, -distance, distance],
            [distance, -distance, -distance], [-distance, distance, distance], [-distance, distance, -distance],
            [-distance, -distance, distance], [-distance, -distance, -distance]
        ], 
        [
            distance, distance, distance, distance, distance, distance, 
            dist2, dist2, dist2, dist2, dist2, dist2, dist2, dist2, dist2, dist2, dist2, dist2, 
            dist3, dist3, dist3, dist3, dist3, dist3, dist3, dist3
        ])

# distance between two points, used as heuristic function during search
def point_distance(point, end):
    return ((point[0] - end[0]) ** 2 + (point[1] - end[1]) ** 2 + (point[2] - end[2]) ** 2) ** 0.5

def has_barrier(barriers, prev_node, curr_node, next_node):
    if (curr_node, next_node) in barriers or \
        (next_node, curr_node) in barriers or \
            (prev_node, curr_node, next_node) in barriers or \
                (next_node, curr_node, prev_node) in barriers:
                return True
    return False

# assuming that found path is invalid in the next deeper layer,
# this function finds all path nodes that can't access the next one
def get_path_barriers(path, lower_start_node, direction_count):
    barriers = []
    for i in range(0, len(path)-1):
        curr_node = path[i][0]
        next_node = path[i+1][0]
        prev_node = None
        if i == 0:
            prev_node = curr_node
            prev_border_nodes = [lower_start_node]
        else:
            prev_node = path[i-1][0]
            # find any non-empty child nodes which 
            # are adjacent to any child nodes of prev node
            prev_border_nodes = curr_node.get_border_nodes(prev_node.children)
        
        # add barrier if there are no child nodes which 
        # are adjacent to any child node of next node
        next_border_nodes = curr_node.get_border_nodes(next_node.children)
        if next_border_nodes == []:
            barriers.append((curr_node, next_node))

        # add barrier if there is no lower-level path 
        # from any prev node adjacent node
        # to any next node adjacent node
        if next_border_nodes != [] and prev_border_nodes != []:
            if not curr_node.can_find_any_path(prev_border_nodes, next_border_nodes, direction_count):
                if prev_node == curr_node:
                    barriers.append((curr_node, next_node))
                else:
                    barriers.append((prev_node, curr_node, next_node))
    return barriers

def trace_path(end_node, start_node):
    path = []
    curr_node = end_node
    if start_node == end_node:
        path.append((start_node, start_node.layer_indexes))
        path.append((end_node, end_node.layer_indexes))
        return path

    while curr_node.prev_node != curr_node:
        path.append((curr_node, curr_node.layer_indexes))
        curr_node = curr_node.prev_node
    path.append((curr_node, curr_node.layer_indexes))
    path.reverse()

    return path

# function that creates a shallow copy
# of the given tree. the copy will only have nodes
# that are included in the given path
def create_tree_from_path(tree, path):
    new_tree = Octree(tree.box, [0], [], layers_left=tree.layers_left)
    for *_, node_indexes in path:
        curr_node_old = tree
        curr_node_new = new_tree
        for i in range(1, len(node_indexes)):
            index = node_indexes[i]
            curr_node_old = curr_node_old.children[index]
            if curr_node_new.children[index] == None:
                curr_node_new.children[index] = Octree(curr_node_old.box, curr_node_old.layer_indexes, curr_node_old.points, parent=curr_node_old.parent, layers_left=curr_node_old.layers_left)
                if i + 1 == len(node_indexes):
                    curr_node_new.children[index].children = curr_node_old.children
            curr_node_new = curr_node_new.children[index]
    return new_tree

########################################
class Octree:
    def __init__(self, box, layer_indexes, points=[], parent=None, layers_left=2):
        self.box = box
        self.points = points
        self.parent = parent
        self.layers_left = layers_left
        self.children = []
        self.layer_indexes = layer_indexes
        for _ in range(8):
            self.children.append(None)

        # A* search data
        self.visited = False
        self.prev_node = None

        # priority func (distance + heuristic)
        self.f = float('inf')

        # distance from start
        self.g = float('inf')

        # heuristic value
        self.h = -1

    def get_heuristic(self, end):
        if self.h == -1:
            self.h = point_distance(self.box.center_point, end)
        return self.h

    def Build(self, prev_layers=[], layer_index=0, debug=True, remove_points_outside_sphere=True):
        new_length = self.box.cube_dim_half / 2

        if self.layers_left < 1 or new_length * 2 < 1:
            return

        prev_layers.append(layer_index)
        if debug:
            print("-")
            print(prev_layers)

        octants = []
        octants.append(BoundaryBox([self.box.center_point[0] - new_length, self.box.center_point[1] - new_length, self.box.center_point[2] - new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] + new_length, self.box.center_point[1] - new_length, self.box.center_point[2] - new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] - new_length, self.box.center_point[1] + new_length, self.box.center_point[2] - new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] + new_length, self.box.center_point[1] + new_length, self.box.center_point[2] - new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] - new_length, self.box.center_point[1] - new_length, self.box.center_point[2] + new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] + new_length, self.box.center_point[1] - new_length, self.box.center_point[2] + new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] - new_length, self.box.center_point[1] + new_length, self.box.center_point[2] + new_length], new_length))
        octants.append(BoundaryBox([self.box.center_point[0] + new_length, self.box.center_point[1] + new_length, self.box.center_point[2] + new_length], new_length))

        points_to_remove_indexes = []
        points_in_children_nodes = []
        points_in_children_is_empty = True
        for _ in range(8):
            points_in_children_nodes.append([])

        m = 0
        printLim = 10000
        start = time.time()
        for i in range(0, len(self.points)):
            point = self.points[i]
            m = m + 1
            if m % printLim == 0:
                end = time.time()

                if debug:
                    print('{} | {:.3f} s'.format(m, end - start))
                start = end
                if printLim <= 800000:
                    printLim = printLim * 10
            for ii in range(0, len(octants)):
                if octants[ii].contains(point):
                    points_to_remove_indexes.append(i)
                    points_in_children_nodes[ii].append(point)
                    if points_in_children_is_empty:
                        points_in_children_is_empty = False
                    break
        if debug:
            end = time.time()
            print('{} | {:.3f} s'.format(m, end - start))
        
        self.points = np.delete(self.points, points_to_remove_indexes, axis=0)
        for i in range(0, len(points_in_children_nodes)):
            if not points_in_children_is_empty:
                new_layers = self.layer_indexes.copy()
                new_layers.append(i)
                self.children[i] = Octree(octants[i], new_layers, points_in_children_nodes[i], self, self.layers_left-1)
            if len(points_in_children_nodes[i]) > 0:
                self.children[i].Build(prev_layers.copy(), i, debug, remove_points_outside_sphere)

    def can_find_any_path(self, nodes_A, nodes_B, direction_count):
        (directions, _) = get_directions(len(self.layer_indexes) + 1, direction_count=direction_count)
        compared_pairs = set()
        for curr_A in nodes_A:
            for curr_B in nodes_B:
                if self.any_path(curr_A, curr_B, directions, compared_pairs):
                    return True
        return False
    
    def any_path(self, node_A, node_B, directions, compared_pairs):
        node_A_point = node_A.box.center_point
        for dir in directions:
            new_point = np.array([node_A_point[0] + dir[0], node_A_point[1] + dir[1], node_A_point[2] + dir[2]])
            new_node = self.find(new_point, len(node_A.layer_indexes))
            if new_node != None and new_node.has_points() and (node_A, new_node) not in compared_pairs:
                compared_pairs.add((node_A, new_node))
                if new_node == node_B:
                    return True
                else:
                    if self.any_path(new_node, node_B, directions, compared_pairs):
                        return True
        return False
        
    def get_border_nodes(self, other_node_children):
        border_nodes = []

        any_child = None
        for child in self.children:
            if child != None:
                any_child = child
                break
        if any_child == None:
            return border_nodes

        max_allowed_distance = any_child.box.cube_dim_half * 2 * 1.1
        for c_child in self.children:
            if c_child == None or not c_child.has_points():
                continue
            for o_child in other_node_children:
                if o_child == None or not o_child.has_points():
                    continue
                dist = point_distance(c_child.box.center_point, o_child.box.center_point)
                if dist <= max_allowed_distance:
                    border_nodes.append(c_child)
                    break

        return border_nodes

    def has_children(self):
        for child in self.children:
            if child != None:
                return True
        return False
    
    def find(self, point, expected_layer, parent=None):
        has_children = False
        if len(self.layer_indexes) <= expected_layer:
            if not self.box.contains(point):
                return None
            parent = self

        for i in range(0, len(self.children)):
            child = self.children[i]
            if child == None:
                continue
            has_children = True
            found_node = None
            found_node = child.find(point, expected_layer, parent)
            if found_node != None:
                return found_node
        
        if not has_children and len(self.points) > 0:
            return parent
        return None
    
    def find_node_by_index(self, indexes):
        target_node = self
        for i in range(0, len(indexes)-1):
            curr_index = indexes[i+1]
            target_node = target_node.children[curr_index]
        return target_node

    def reset_all_pathfinding_data(self):
        self.prev_node = None
        self.f = float('inf')
        self.g = float('inf')
        self.h = -1
        self.visited = False

        for child in self.children:
            if child != None:
                child.reset_all_pathfinding_data()

    def reset_layer_pathfinding_data(self, expected_layer):
        if len(self.layer_indexes) == expected_layer:
            self.prev_node = None
            self.f = float('inf')
            self.g = float('inf')
            self.visited = False

        if len(self.layer_indexes) < expected_layer:
            for child in self.children:
                if child != None:
                    child.reset_layer_pathfinding_data(expected_layer)

    def has_points(self):
        if len(self.points) > 0:
            return True
        for c in self.children:
            if c != None:
                if c.has_points() == True:
                    return True
        return False

    def find_shortest_path(self, start, end, root=None, expected_layers=2, start_layers=-1, all_barriers=[], direction_count=6):
        first_search = root == None
        if first_search:
            root = self
            start_layers = expected_layers
            self.reset_all_pathfinding_data()
        
        start_node = self.find(start, expected_layers)
        end_node = self.find(end, expected_layers)
        if start_node == None or end_node == None:
            print("Start or end is invalid")
            return ([], None)
        if start[0] == end[0] and start[1] == end[1] and start[2] == end[2]:
            print("Start and end are equal")
            return ([], None)
        
        search_start = True
        start_node.f = 0
        start_node.g = 0
        start_node.h = -1
        start_node.prev_node = start_node
        open_list = []
        heapq.heappush(open_list, (0.0, start_node.layer_indexes))
        
        (directions, distances) = get_directions(expected_layers, end_node.box.cube_dim_half * 2, direction_count)
        while len(open_list) > 0:
            curr_node = self.find_node_by_index(heapq.heappop(open_list)[1])
            curr_node.visited = True
            curr_node_point = curr_node.box.center_point

            for i in range(0, len(directions)):
                prev_node = curr_node.prev_node

                # check start node first
                if search_start:
                    i = i - 1
                    new_node = curr_node
                    new_node.visited = False
                else:
                    (dir, dist) = (directions[i], distances[i])
                    new_point = np.array([curr_node_point[0] + dir[0], curr_node_point[1] + dir[1], curr_node_point[2] + dir[2]])
                    new_node = self.find(new_point, expected_layers)

                                
                # check if new node exists, is not empty and doesn't have barrier
                if new_node != None and not new_node.visited and new_node.has_points() and not has_barrier(all_barriers, prev_node, curr_node, new_node):
                    if new_node == end_node:
                        new_node.prev_node = curr_node
                        path = trace_path(new_node, start_node)

                        if not new_node.has_children():
                            return (path, None)
                        else:
                            tree_subset = create_tree_from_path(root, path)
                            (deeper_path, lower_start_node) = tree_subset.find_shortest_path(start, end, root=root, expected_layers=expected_layers + 1, start_layers=start_layers, all_barriers=all_barriers, direction_count=direction_count)
                            
                            # check if deeper path was not found due to start / end error
                            if deeper_path == [] and lower_start_node == None:
                                return ([], None)

                            if deeper_path == [] and lower_start_node != None:
                                # find path barriers and restart search in this layer
                                found_new_barriers = False
                                found_barriers = get_path_barriers(path, lower_start_node, direction_count)

                                for barrier in found_barriers:
                                    if barrier not in all_barriers:
                                        found_new_barriers = True
                                        all_barriers.append(barrier)

                                if found_new_barriers:
                                    root.reset_layer_pathfinding_data(expected_layers)
                                    start_node.f = 0
                                    start_node.g = 0
                                    start_node.h = -1
                                    start_node.prev_node = start_node
                                    open_list = []
                                    heapq.heappush(open_list, (0.0, start_node.layer_indexes))
                            else:
                                return (deeper_path, None)
                    elif not search_start:
                        # when new node is not in priority list
                        # or when a quicker path to it is found,
                        # new node is added to priority list
                        g_new = curr_node.g + dist
                        if g_new < new_node.g:
                            new_node.g = g_new
                            new_node.f = g_new + new_node.get_heuristic(end)
                            new_node.prev_node = curr_node
                            heapq.heappush(open_list, (new_node.f, new_node.layer_indexes))
                if search_start:
                    search_start = False
                    new_node.visited = True
        
        # path was not found in this layer
        # returned start node implies that other path may be found
        # and is used when creating barriers in upper layer
        root.reset_layer_pathfinding_data(expected_layers)
        return ([], start_node)
    
    def add(self, point):
        for child in self.children:
            if child == None:
                continue
            if child.add(point):
                return True
        
        if self.box.contains(point) != 2:
            return False
        
        self.points = np.vstack((self.points, point))
        return True
