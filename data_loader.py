import os
import numpy as np
import open3d as o3d


def load_point_cloud_from_csv(filename):
    data = np.loadtxt(filename, delimiter=',')
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(data))


def load_point_clouds_from_sensors(directory, sensor_ids, scan_numbers):
    all_files = [f for f in os.listdir(directory) if f.endswith('.csv')]
    filenames = []
    for filename in all_files:
        for sensor_id in sensor_ids:
            if f'sensor_{sensor_id}_' in filename:
                for scan_number in scan_numbers:
                    if f'_{scan_number:02d}.csv' in filename:
                        filenames.append(os.path.join(directory, filename))
                        break
                break
    return filenames


# Read the trajectories from the pitt_trajectories.csv file.
def load_trajectories(file_path):
    return np.loadtxt(file_path, delimiter=',', skiprows=1)
