import numpy as np

class BoundaryBox:
    def __init__(self, center_point, cube_dim_half):
        self.center_point = np.array(center_point) 
        self.cube_dim_half = cube_dim_half 
        self.boundary_points = []
        self.boundary_points.append([center_point[0] - cube_dim_half, center_point[1] - cube_dim_half, center_point[2] - cube_dim_half])
        self.boundary_points.append([center_point[0] + cube_dim_half, center_point[1] - cube_dim_half, center_point[2] - cube_dim_half])
        self.boundary_points.append([center_point[0] - cube_dim_half, center_point[1] + cube_dim_half, center_point[2] - cube_dim_half])
        self.boundary_points.append([center_point[0] + cube_dim_half, center_point[1] + cube_dim_half, center_point[2] - cube_dim_half])
        self.boundary_points.append([center_point[0] - cube_dim_half, center_point[1] - cube_dim_half, center_point[2] + cube_dim_half])
        self.boundary_points.append([center_point[0] + cube_dim_half, center_point[1] - cube_dim_half, center_point[2] + cube_dim_half])
        self.boundary_points.append([center_point[0] - cube_dim_half, center_point[1] + cube_dim_half, center_point[2] + cube_dim_half])
        self.boundary_points.append([center_point[0] + cube_dim_half, center_point[1] + cube_dim_half, center_point[2] + cube_dim_half])
    
    def contains(self, point):
        if point[0] > self.boundary_points[7][0] or point[0] < self.boundary_points[0][0]:
            return False
        if point[1] > self.boundary_points[7][1] or point[1] < self.boundary_points[0][1]:
            return False
        if point[2] > self.boundary_points[7][2] or point[2] < self.boundary_points[0][2]:
            return False
        return True
    
    def bounds_cube(self, cube_density):
        points = []
        for i in range(0, int(self.cube_dim_half * 2), cube_density):
            points.append([sum(x) for x in zip(self.center_point, [-self.cube_dim_half, -self.cube_dim_half, i - self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [self.cube_dim_half, -self.cube_dim_half, i - self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [-self.cube_dim_half, self.cube_dim_half, i - self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [self.cube_dim_half, self.cube_dim_half, i - self.cube_dim_half])])

            points.append([sum(x) for x in zip(self.center_point, [-self.cube_dim_half, i - self.cube_dim_half, -self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [self.cube_dim_half, i - self.cube_dim_half, -self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [-self.cube_dim_half, i - self.cube_dim_half, self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [self.cube_dim_half, i - self.cube_dim_half, self.cube_dim_half])])

            points.append([sum(x) for x in zip(self.center_point, [i - self.cube_dim_half, -self.cube_dim_half, -self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [i - self.cube_dim_half, self.cube_dim_half, -self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [i - self.cube_dim_half, -self.cube_dim_half, self.cube_dim_half])])
            points.append([sum(x) for x in zip(self.center_point, [i - self.cube_dim_half, self.cube_dim_half, self.cube_dim_half])])
        return points