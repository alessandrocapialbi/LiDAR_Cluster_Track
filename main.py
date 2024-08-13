from downsample_pcd import downsample_pcd
from sensor_selection import select_sensors
from data_loader import *
from transform_coordinates import *
import open3d as o3d
from point_cloud_merger import merge_point_clouds
from clustering import dbscan_clustering, create_bounding_boxes
from simulation import update_visualization
import time

current_directory = os.path.dirname(os.path.abspath(__file__))
point_cloud_directory = os.path.join(current_directory, 'filtered_sensors_data')
sensors_positions_path = os.path.join(current_directory, 'sensors_positions/pitt_sensor_positions.csv')

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
sensors_positions_df = load_sensor_positions(sensors_positions_path)

centroid = calculate_sensors_centroid(sensors_positions_df)

# Create a visualizer instance
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()

# Loop through scan indices from 20 to 70
for i in range(20, 71):

    point_clouds = []
    # List to store the combined point cloud
    combined_geometries = []

    bounding_boxes = []

    # Load the point clouds from the selected sensors
    sensors_scans = load_point_clouds_from_sensors(point_cloud_directory, selected_sensors, i)
    print(sensors_scans)

    # Load and transform scans for each selected sensor
    for sensor_id, sensor_scan in zip(selected_sensors, sensors_scans):
        transformed_xyz = load_and_transform_scan(sensor_scan, sensors_positions_df, centroid, sensor_id)
        if transformed_xyz is not None:
            print(f"Loading scan {i} for sensor {sensor_id}")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(transformed_xyz)
            point_clouds.append(pcd)

    # Merge the point clouds
    pcd_combined = merge_point_clouds(point_clouds)
    print("Number of points: ", len(pcd_combined.points))

    # Downsample the point cloud
    pcd_combined = downsample_pcd(pcd_combined, voxel_size=0.3)
    print("Number of points after downsampling: ", len(pcd_combined.points))

    # Perform DBSCAN clustering and create bounding boxes
    clusters, labels = dbscan_clustering(pcd_combined, plot_k_distance=True)
    bounding_boxes = create_bounding_boxes(clusters)

    # Update the visualization
    update_visualization(vis, pcd_combined, bounding_boxes)

    # Pause for a short time to simulate time evolution
    time.sleep(2)

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

"""""
sensors_points = np.genfromtxt(sensors_positions_path, delimiter=',', skip_header=1, usecols=[0, 1, 2])
sensors_geometry = o3d.geometry.PointCloud()
sensors_geometry.points = o3d.utility.Vector3dVector(sensors_points)
o3d.visualization.draw_geometries([sensors_geometry])
"""""

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
