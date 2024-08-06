import os
import numpy as np
import open3d as o3d
from sensor_selection import select_sensors
from data_loader import load_point_clouds_from_sensors
from downsample_pcd import downsample_pcd

current_directory = os.path.dirname(os.path.abspath(__file__))
point_cloud_directory = os.path.join(current_directory, 'filtered_sensors_data')

# Choose the total number of sensors available
print("Enter the total number of sensors available: ")
try:
    num_sensors = int(input())
except ValueError:
    print("Please enter a valid number.")
    exit(1)

selected_sensors = select_sensors(num_sensors)

# Load the point clouds from the selected sensors

sensors_scans = load_point_clouds_from_sensors(point_cloud_directory, selected_sensors, 20)
pcd = o3d.geometry.PointCloud()
for sensor_scan in sensors_scans:
    data = np.genfromtxt(sensor_scan, delimiter=',', skip_header=1, usecols=[5, 6, 7])
    pcd.points = o3d.utility.Vector3dVector(data)
    pcd = downsample_pcd(pcd, 0.3)

'''''''''
filename = "C:\\Users\\aless\\Desktop\\LiDARClusterTrack\\alessandrocapialbi\\LiDAR_Cluster_Track\\filtered_sensors_data\\sensor_0_24.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=1, usecols=[5, 6, 7])

pcd = o3d.geometry.PointCloud()
print("Numero di punti: ", len(data))
pcd.points = o3d.utility.Vector3dVector(data)
pcd = pcd.voxel_down_sample(voxel_size=0.3)
print("Numero di punti dopo il down sampling: ", len(pcd.points))


# Visualizza la nuvola di punti
o3d.visualization.draw_geometries([pcd])

'''''''''''
