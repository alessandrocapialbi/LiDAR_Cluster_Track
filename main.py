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

current_directory = os.path.dirname(os.path.abspath(__file__))
point_cloud_directory = os.path.join(current_directory, 'filtered_sensors_data')
sensors_positions_path = os.path.join(current_directory, 'sensors_positions/pitt_sensor_positions.csv')
trajectories_path = os.path.join(current_directory, 'trajectories/pitt_trajectories.csv')
predicted_trajectories_file_path = os.path.join(current_directory, 'output/predicted_trajectories.csv')
video_frames_directory = os.path.join(current_directory, 'output/video_frames')

# Remove the file if it exists
if os.path.exists(predicted_trajectories_file_path):
    os.remove(predicted_trajectories_file_path)

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

predicted_trajectories = {}

vehicle_colors = {}

# Load the real trajectories from trajectories_df
real_trajectories = {}


# Function to generate a random color
def generate_random_color():
    return np.random.uniform(0, 1, 3)


# Initialize the frame index for saving screenshots and create a video
frame_index = 0

# Loop through scan indices from 20 to 70
for i in range(20, 71):
    # Filter the trajectories for the current timestep
    current_trajectories = trajectories_df[trajectories_df['time'] == i]
    for _, row in current_trajectories.iterrows():
        vehicle_id = int(row['label']) if row['label'] != 'AV' else -1
        point = [row['x'], row['y']]

        # Transform the trajectory point
        transformed_point = np.array(point) - centroid[:2]
        # Check if this vehicle_id already exists in the dictionary
        if vehicle_id not in real_trajectories:
            real_trajectories[vehicle_id] = [transformed_point]  # Start a list for this vehicle
        else:
            real_trajectories[vehicle_id].append(transformed_point)  # Append the point

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
        clusters, labels = dbscan_clustering(pcd_combined, i)
        bounding_boxes, bbox_centroids = create_bounding_boxes(clusters, i)
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
                    if prev_id not in predicted_trajectories:
                        predicted_trajectories[prev_id] = []

                    # Append the current centroid to the trajectory
                    predicted_trajectories[prev_id].append(current_centroid)

            # Extract only the X and Y coordinates of the predicted centroids
            predicted_trajectories_xy = {vehicle_id: [point[:2] for point in points] for vehicle_id, points in
                                         predicted_trajectories.items()}
            calculate_mse(predicted_trajectories_xy, real_trajectories, tracking_threshold, i - 20)

        # Save predicted trajectories to CSV
        trajectory_data = []
        for vehicle_id, points in predicted_trajectories.items():
            for point in points:
                trajectory_data.append([i, vehicle_id, *point])

        df_trajectories = pd.DataFrame(trajectory_data, columns=['scan', 'vehicle_id', 'x', 'y', 'z'])
        df_trajectories.to_csv(predicted_trajectories_file_path, mode='a', header=False, index=False)

        # Draw the trajectories
        # Combine real and predicted trajectories
        # If we want to visualize only the predicted trajectories, we can use only the 'trajectories' dictionary and vice versa
        all_trajectories = {**predicted_trajectories, **real_trajectories}

        # Draw the trajectories
        trajectory_lines = []
        for vehicle_id, points in all_trajectories.items():
            if len(points) > 1:  # Ensure at least two timesteps
                # Check if points are 2D or 3D
                if len(points[0]) == 2:
                    # Convert 2D points to 3D by adding a z coordinate (set to 0)
                    points_3d = np.hstack((np.asarray(points), np.zeros((len(points), 1))))
                else:
                    points_3d = np.asarray(points)

                if vehicle_id not in vehicle_colors:
                    vehicle_colors[vehicle_id] = generate_random_color()

                color = vehicle_colors[vehicle_id]
                for idx in range(len(points_3d) - 1):
                    point1 = points_3d[idx]
                    point2 = points_3d[idx + 1]
                    # Create a cylinder between the two points
                    cylinder = create_cylinder_between_points(point1, point2, color, radius=0.1)
                    trajectory_lines.append(cylinder)

        # Update the visualization including trajectories
        update_visualization(vis, pcd_combined, bounding_boxes + trajectory_lines)


        screenshot_path = os.path.join(video_frames_directory, f"frame_{frame_index:04d}.png")
        vis.capture_screen_image(screenshot_path)
        frame_index += 1


        # Update the previous bounding boxes and centroids
        prev_ids = bbox_ids
        prev_bbox_centroids = bbox_centroids

    time.sleep(0.1)

# Close the visualizer
vis.destroy_window()
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
