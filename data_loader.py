import os
import numpy as np
import open3d as o3d


def load_point_cloud_from_csv(filename):
    data = np.loadtxt(filename, delimiter=',')
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data))


# Load the point clouds from the sensors one by one.
def load_point_clouds_from_sensors(directory, sensor_ids, scan_number):
    filenames = []
    for sensor_id in sensor_ids:
        filename = f'sensor_{sensor_id}_{scan_number:02d}.csv'
        file_path = os.path.join(directory, filename)
        if os.path.exists(file_path):
            filenames.append(file_path)
    return filenames


# Read the trajectories from the pitt_trajectories.csv file.
def load_trajectories(file_path):
    return np.loadtxt(file_path, delimiter=',', skiprows=1)
