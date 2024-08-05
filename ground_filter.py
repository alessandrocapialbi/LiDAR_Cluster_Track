import numpy as np
import open3d as o3d


# All the z points below the threshold are considered as ground points.
def filter_ground(points, z_threshold):
    filtered_points = points[points[:, 2] > z_threshold]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(filtered_points)
    return pcd
