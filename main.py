import time
from sensor_selection import select_sensors
from data_loader import *
from transform_coordinates import *
from clustering import *
from simulation import update_visualization
from tracking import track_vehicles, calculate_threshold
import pandas as pd

current_directory = os.path.dirname(os.path.abspath(__file__))
point_cloud_directory = os.path.join(current_directory, 'filtered_sensors_data')
sensors_positions_path = os.path.join(current_directory, 'sensors_positions/pitt_sensor_positions.csv')
trajectories_path = os.path.join(current_directory, 'trajectories/pitt_trajectories.csv')

frequency = 10

# Choose the total number of sensors available
print("Enter the total number of sensors available: ")
try:
    num_sensors = int(input())
except ValueError:
    print("Please enter a valid number.")
    exit(1)

# Select the sensors to use
selected_sensors = select_sensors(num_sensors)

# Load the sensors positions
sensors_positions_df = load_file(sensors_positions_path)

# Load the trajectories
trajectories_df = load_file(trajectories_path)

# Calculate the threshold for the tracking algorithm
tracking_threshold = calculate_threshold(trajectories_df, frequency, percentage_margin=25)

# Calculate the centroid of the sensors
centroid = calculate_sensors_centroid(sensors_positions_df)

vis = o3d.visualization.Visualizer()
vis.create_window()

prev_ids = []
prev_bbox_centroids = []

# Loop through scan indices from 20 to 70
for i in range(20, 71):

    point_clouds = []
    # List to store the combined point cloud
    combined_geometries = []

    bounding_boxes = []

    all_transformed_xyz = []

    all_object_ids = []

    # Load the point clouds from the selected sensors
    sensors_scans = load_point_clouds_from_sensors(point_cloud_directory, selected_sensors, i)
    print(sensors_scans)

    # Load and transform scans for each selected sensor
    for sensor_id, sensor_scan in zip(selected_sensors, sensors_scans):
        print(f"Loading scan {i} for sensor {sensor_id}")
        transformed_xyz, object_ids = load_and_transform_scan(sensor_scan, sensors_positions_df, centroid, sensor_id)
        all_transformed_xyz.append(transformed_xyz)
        all_object_ids.extend(object_ids)

    # Convert to numpy arrays for easier handling
    all_transformed_xyz = np.vstack(all_transformed_xyz)

    if all_transformed_xyz is not None:
        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(all_transformed_xyz)
        print("Number of points before downsampling: ", len(pcd_combined.points))
        pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.3)
        print("Number of points after downsampling: ", len(pcd_combined.points))

        # Perform DBSCAN clustering and create bounding boxes
        clusters, labels = dbscan_clustering(pcd_combined)
        bounding_boxes, bbox_centroids = create_bounding_boxes(clusters)
        bbox_ids = associate_ids_to_bboxes(bbox_centroids, all_object_ids, all_transformed_xyz)

        # Track vehicles between scans considering the bounding box centroids
        if prev_bbox_centroids:
            matches, exited_vehicles, entered_vehicles = track_vehicles(prev_bbox_centroids, bbox_centroids, prev_ids,
                                                                        bbox_ids,
                                                                        tracking_threshold, frequency)
            print("Matches:\n", pd.DataFrame(matches))
            print("Exited vehicles:", pd.DataFrame(exited_vehicles))
            print("Newly entered vehicles:", pd.DataFrame(entered_vehicles))

            # Update the visualization including trajectories
            update_visualization(vis, pcd_combined, bounding_boxes)

        # Update the previous bounding boxes and centroids
        prev_ids = bbox_ids
        prev_bbox_centroids = bbox_centroids

        time.sleep(0.1)

# Close the visualizer
vis.destroy_window()

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

"""""
sensors_points = np.genfromtxt(sensors_positions_path, delimiter=',', skip_header=1, usecols=[0, 1, 2])
sensors_geometry = o3d.geometry.PointCloud()
sensors_geometry.points = o3d.utility.Vector3dVector(sensors_points)
o3d.visualization.draw_geometries([sensors_geometry])
"""""""

'''''''''
points = np.asarray(pcd_combined.points)
print("Cloud point size", points.shape)

dbscan = DBSCAN(eps=0.5, min_samples=10)
clusters = dbscan.fit_predict(points)

unique_labels = np.unique(clusters)
print("Cluster labels", unique_labels)

cmap = plt.colormaps['tab10']  # Accesso alla mappa di colori 'tab10'

# Crea una lista per gli oggetti di geometria da visualizzare
geometries = [pcd_combined]

# Calcola la bounding box per ogni cluster
for label in unique_labels:
    if label == -1:
        # Skip noise points
        continue

    # Trova i punti appartenenti al cluster corrente
    cluster_points = points[clusters == label]

    # Crea una point cloud per il cluster corrente
    cluster_pcd = o3d.geometry.PointCloud()
    cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)

    # Calcola la bounding box del cluster
    bbox = cluster_pcd.get_axis_aligned_bounding_box()
    color = cmap(label / len(unique_labels))[:3]  # Assegna solo RGB
    bbox.color = color

    # Aggiungi la bounding box alla lista delle geometrie
    geometries.append(bbox)

# Visualizza la point cloud e le bounding box
o3d.visualization.draw_geometries(geometries)
'''''''''

"""""

"""""
import numpy as np
import open3d as o3d
import os

current_directory = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(current_directory, 'filtered_sensors_data/sensor_3_20.csv')

data = np.genfromtxt(filename, delimiter=',', skip_header=1, usecols=[5, 6, 7])

pcd = o3d.geometry.PointCloud()
print("Numero di punti: ", len(data))
pcd.points = o3d.utility.Vector3dVector(data)

# Visualizza la nuvola di punti
o3d.visualization.draw_geometries([pcd])

"""""
