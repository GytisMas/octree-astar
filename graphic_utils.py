import numpy as np
import open3d as o3d

def create_vertical_line(point, height, color):
    points = []
    for i in range(0, int(height * 2), 100):
        points.append([sum(x) for x in zip(point, [0, 0, i - height])])
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(points)
    geom.paint_uniform_color(color)
    return geom

def create_bounds_cube(box, color, cube_density):
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(box.bounds_cube(cube_density))
    geom.paint_uniform_color(color)
    return geom

def create_geometry(node, cube_density, sphere_density, geometries=[], root=True):
    if node == None:
        return
    color = [0, 0, 0]
    if not root:
        color = np.random.rand(1,3)[0]

    
    for child in node.children:
        child_geom = create_geometry(child, cube_density, sphere_density, geometries, False)

    if len(node.points) < 1:
        return

    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(node.points)
    if not np.equal(color,[0, 0, 0]).all(0):
        geom.paint_uniform_color(color)
    geometries.append(geom)