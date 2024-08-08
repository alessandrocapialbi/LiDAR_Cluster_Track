import open3d as o3d
import numpy as np


def merge_point_clouds(point_clouds):
    pcd_combined = o3d.geometry.PointCloud()
    combined_points = np.vstack([np.asarray(pcd.points) for pcd in point_clouds])
    pcd_combined.points = o3d.utility.Vector3dVector(combined_points)
    return pcd_combined
