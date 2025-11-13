import time
from sensor_selection import select_sensors
from data_loader import *
from transform_coordinates import *
from clustering import *
from simulation import update_visualization, create_cylinder_between_points
from tracking import track_vehicles, calculate_threshold, calculate_mse
import pandas as pd
import os
import numpy as np
import open3d as o3d

# Get the current directory path
current_directory = os.path.dirname(os.path.abspath(__file__))

# Define paths for input and output files
point_cloud_directory = os.path.join(current_directory, 'filtered_sensors_data')
sensors_positions_path = os.path.join(current_directory, 'sensors_positions/pitt_sensor_positions.csv')
trajectories_path = os.path.join(current_directory, 'trajectories/pitt_trajectories.csv')
predicted_trajectories_file_path = os.path.join(current_directory, 'output/predicted_trajectories.csv')
video_frames_directory = os.path.join(current_directory, 'output/video_frames')

# Remove the predicted trajectories file if it already exists
if os.path.exists(predicted_trajectories_file_path):
    os.remove(predicted_trajectories_file_path)

# Define the processing frequency in Hz
frequency = 10

# Ask the user to specify how many sensors are available
print("Enter the total number of sensors available: ")
try:
    num_sensors = int(input())
except ValueError:
    print("Please enter a valid number.")
    exit(1)

# Select which sensors to use from the available ones
selected_sensors = select_sensors(num_sensors)

# Load the CSV file containing sensors positions
sensors_positions_df = load_file(sensors_positions_path)

# Load the CSV file containing the true trajectories
trajectories_df = load_file(trajectories_path)

# Compute the distance threshold used by the tracking algorithm
tracking_threshold = calculate_threshold(trajectories_df, frequency, percentage_margin=25)

# Compute the centroid of all sensors in the environment
centroid = calculate_sensors_centroid(sensors_positions_df)

# Initialize the Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()

# Initialize lists and dictionaries for tracking
prev_ids = []
prev_bbox_centroids = []
predicted_trajectories = {}
vehicle_colors = {}
real_trajectories = {}

# Generate a random RGB color
def generate_random_color():
    return np.random.uniform(0, 1, 3)

# Initialize the frame index used for saving screenshots
frame_index = 0

# Main processing loop for scan indices from 20 to 70
for i in range(20, 71):

    # Select the trajectories corresponding to the current timestep
    current_trajectories = trajectories_df[trajectories_df['time'] == i]

    # Update the dictionary of real trajectories
    for _, row in current_trajectories.iterrows():
        vehicle_id = int(row['label']) if row['label'] != 'AV' else -1
        point = [row['x'], row['y']]
        transformed_point = np.array(point) - centroid[:2]
        if vehicle_id not in real_trajectories:
            real_trajectories[vehicle_id] = [transformed_point]
        else:
            real_trajectories[vehicle_id].append(transformed_point)

    # Initialize lists for storing all sensor scans and results
    point_clouds = []
    combined_geometries = []
    bounding_boxes = []
    all_transformed_xyz = []
    all_object_ids = []
    predicted_centroids = []

    # Load the LiDAR scans for the selected sensors at timestep i
    sensors_scans = load_point_clouds_from_sensors(point_cloud_directory, selected_sensors, i)
    print(sensors_scans)

    # For each selected sensor, load and transform its scan to the global coordinate system
    for sensor_id, sensor_scan in zip(selected_sensors, sensors_scans):
        print(f"Loading scan {i} for sensor {sensor_id}")
        transformed_xyz, object_ids = load_and_transform_scan(sensor_scan, sensors_positions_df, centroid, sensor_id)
        all_transformed_xyz.append(transformed_xyz)
        all_object_ids.extend(object_ids)

    # Merge all transformed point clouds into a single numpy array
    all_transformed_xyz = np.vstack(all_transformed_xyz)

    if all_transformed_xyz is not None:
        # Create a single Open3D point cloud with all points
        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(all_transformed_xyz)

        # Apply voxel downsampling to reduce point cloud density
        print("Number of points before downsampling: ", len(pcd_combined.points))
        pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.3)
        print("Number of points after downsampling: ", len(pcd_combined.points))

        # Perform DBSCAN clustering to group nearby points
        clusters, labels = dbscan_clustering(pcd_combined, i)

        # Create bounding boxes around each cluster and compute their centroids
        bounding_boxes, bbox_centroids = create_bounding_boxes(clusters, i)

        # Associate each bounding box with an object ID
        bbox_ids = associate_ids_to_bboxes(bbox_centroids, all_object_ids, all_transformed_xyz)

        # Perform vehicle tracking by comparing the current and previous centroids
        if prev_bbox_centroids:
            matches, exited_vehicles, entered_vehicles, predicted_centroids = track_vehicles(
                prev_bbox_centroids,
                bbox_centroids,
                prev_ids,
                bbox_ids,
                tracking_threshold,
                frequency
            )

            print("Matches:\n", pd.DataFrame(matches))
            print("Exited vehicles:", pd.DataFrame(exited_vehicles))
            print("Newly entered vehicles:", pd.DataFrame(entered_vehicles))

            # Update the predicted trajectories using the matches between frames
            for prev_id, current_id in matches:
                if current_id in bbox_ids:
                    current_centroid = bbox_centroids[bbox_ids.index(current_id)]
                    if prev_id not in predicted_trajectories:
                        predicted_trajectories[prev_id] = []
                    predicted_trajectories[prev_id].append(current_centroid)

            # Extract only the XY coordinates from predicted trajectories
            predicted_trajectories_xy = {
                vehicle_id: [point[:2] for point in points]
                for vehicle_id, points in predicted_trajectories.items()
            }

            # Compute the mean squared error between predicted and real trajectories
            calculate_mse(predicted_trajectories_xy, real_trajectories, tracking_threshold, i - 20)

        # Save the predicted trajectories to CSV
        trajectory_data = []
        for vehicle_id, points in predicted_trajectories.items():
            for point in points:
                trajectory_data.append([i, vehicle_id, *point])
        df_trajectories = pd.DataFrame(trajectory_data, columns=['scan', 'vehicle_id', 'x', 'y', 'z'])
        df_trajectories.to_csv(predicted_trajectories_file_path, mode='a', header=False, index=False)

        # Combine both real and predicted trajectories for visualization
        all_trajectories = {**predicted_trajectories, **real_trajectories}

        # Create 3D cylinders representing the trajectories of vehicles
        trajectory_lines = []
        for vehicle_id, points in all_trajectories.items():
            if len(points) > 1:
                if len(points[0]) == 2:
                    points_3d = np.hstack((np.asarray(points), np.zeros((len(points), 1))))
                else:
                    points_3d = np.asarray(points)

                if vehicle_id not in vehicle_colors:
                    vehicle_colors[vehicle_id] = generate_random_color()

                color = vehicle_colors[vehicle_id]
                for idx in range(len(points_3d) - 1):
                    point1 = points_3d[idx]
                    point2 = points_3d[idx + 1]
                    cylinder = create_cylinder_between_points(point1, point2, color, radius=0.1)
                    trajectory_lines.append(cylinder)

        # Update the visualization with current point cloud, bounding boxes, and trajectories
        update_visualization(vis, pcd_combined, bounding_boxes + trajectory_lines)

        # Capture a screenshot of the current frame
        screenshot_path = os.path.join(video_frames_directory, f"frame_{frame_index:04d}.png")
        vis.capture_screen_image(screenshot_path)
        frame_index += 1

        # Store the current centroids and IDs for the next iteration
        prev_ids = bbox_ids
        prev_bbox_centroids = bbox_centroids

    # Wait briefly before processing the next frame
    time.sleep(0.1)

# Close the visualization window at the end of the process
vis.destroy_window()

# Set Open3D verbosity level back to default
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
