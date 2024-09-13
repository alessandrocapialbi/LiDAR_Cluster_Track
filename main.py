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

trajectories = {}

vehicle_colors = {}


# Loop through scan indices from 20 to 70
def generate_random_color():
    return np.random.uniform(0, 1, 3)


for i in range(20, 71):

    point_clouds = []
    combined_geometries = []
    bounding_boxes = []
    all_transformed_xyz = []
    all_object_ids = []
    predicted_centroids = []

    # Load the point clouds from the selected sensors
    sensors_scans = load_point_clouds_from_sensors(point_cloud_directory, selected_sensors, i)
    print(sensors_scans)

    # Load and transform scans for each selected sensor
    for sensor_id, sensor_scan in zip(selected_sensors, sensors_scans):
        print(f"Loading scan {i} for sensor {sensor_id}")
        transformed_xyz, object_ids = load_and_transform_scan(sensor_scan, sensors_positions_df, centroid,
                                                              sensor_id)
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
            matches, exited_vehicles, entered_vehicles, predicted_centroids = track_vehicles(prev_bbox_centroids,
                                                                                             bbox_centroids,
                                                                                             prev_ids,
                                                                                             bbox_ids,
                                                                                             tracking_threshold,
                                                                                             frequency)
            print("Matches:\n", pd.DataFrame(matches))
            print("Exited vehicles:", pd.DataFrame(exited_vehicles))
            print("Newly entered vehicles:", pd.DataFrame(entered_vehicles))

            for prev_id, current_id in matches:
                if current_id in bbox_ids:
                    current_centroid = bbox_centroids[bbox_ids.index(current_id)]
                    # If the vehicle ID is not in the dictionary, add it
                    if prev_id not in trajectories:
                        trajectories[prev_id] = []

                    # Append the current centroid to the trajectory
                    trajectories[prev_id].append(current_centroid)

        # Draw the trajectories
        trajectory_lines = []
        for vehicle_id, points in trajectories.items():
            if len(points) > 1:
                lines = np.asarray([[i, i + 1] for i in range(len(points) - 1)], dtype=np.int32)
                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(np.asarray(points))
                line_set.lines = o3d.utility.Vector2iVector(lines)
                if vehicle_id not in vehicle_colors:
                    vehicle_colors[vehicle_id] = generate_random_color()
                colors = np.array([vehicle_colors[vehicle_id] for _ in range(len(lines))], dtype=np.float64)
                line_set.colors = o3d.utility.Vector3dVector(colors)
                trajectory_lines.append(line_set)

        # Update the visualization including trajectories
        update_visualization(vis, pcd_combined, bounding_boxes + trajectory_lines)

        # Update the previous bounding boxes and centroids
        prev_ids = bbox_ids
        prev_bbox_centroids = bbox_centroids

    time.sleep(0.1)

# Close the visualizer
vis.destroy_window()
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
