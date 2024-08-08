from sensor_selection import select_sensors
from data_loader import *
from transform_coordinates import *
import open3d as o3d
from point_cloud_merger import merge_point_clouds

current_directory = os.path.dirname(os.path.abspath(__file__))
point_cloud_directory = os.path.join(current_directory, 'filtered_sensors_data')
sensors_positions_path = os.path.join(current_directory, 'sensors_positions\\pitt_sensor_positions.csv')

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
print(sensors_scans)

sensors_positions_df = load_sensor_positions(sensors_positions_path)
print(sensors_positions_df)

center_sensors = calculate_sensors_centroid(sensors_positions_df)
print(center_sensors)

point_clouds = []
pcd_combined = o3d.geometry.PointCloud()
'''''''''
For every imported scan of the selected sensors transform the coordinates and create the new point cloud 
with global coordinates 
'''''''''
for sensor_id, file_path in zip(selected_sensors, sensors_scans):
    transformed_data = load_and_transform_scan(file_path, sensors_positions_df, center_sensors, sensor_id)
    print(file_path)
    print("\n")
    print(sensor_id)
    print("\n")
    print(transformed_data)
    print("\n")
    if transformed_data is not None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(transformed_data)
        point_clouds.append(pcd)

# Merge the point clouds
pcd_combined = merge_point_clouds(point_clouds)
print("Number of points: ", len(pcd_combined.points))

pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.3)
print("Number of points after the downsampling", len(pcd_combined.points))

# View the combined point cloud
o3d.visualization.draw_geometries([pcd_combined])

'''''''''
import numpy as np
import open3d as o3d

filename = "C:\\Users\\aless\\Desktop\\LiDARClusterTrack\\alessandrocapialbi\\LiDAR_Cluster_Track\\filtered_sensors_data\\sensor_2_20.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=1, usecols=[5, 6, 7])

pcd = o3d.geometry.PointCloud()
print("Numero di punti: ", len(data))
pcd.points = o3d.utility.Vector3dVector(data)

# Visualizza la nuvola di punti
o3d.visualization.draw_geometries([pcd])


'''''''''
