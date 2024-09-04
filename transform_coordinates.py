import os
import numpy as np


def rotation_matrix_from_euler_angles(angles):
    """
    Create a rotation matrix from Euler angles (roll, pitch, yaw) in radians.

    Args:
    - angles (list or np.array): Array of three Euler angles [roll, pitch, yaw] in radians.

    Returns:
    - R (np.ndarray): Rotation matrix.
    """
    roll, pitch, yaw = angles

    # Rotation matrix around X-axis (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Rotation matrix around Y-axis (pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation matrix around Z-axis (yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix (Z * Y * X)
    R = R_z @ R_y @ R_x

    return R


def transform_coordinates(xyz, origin_sensor, center_sensors, R):
    # Transform the coordinates of the point cloud
    origin_global = origin_sensor - center_sensors
    transformed_xyz = xyz + origin_global
    transformed_xyz = np.dot(transformed_xyz, R)
    return transformed_xyz


def calculate_sensors_centroid(sensor_positions_df):
    # Calculate the centroid of the sensors
    center_sensors = sensor_positions_df[['x', 'y', 'z']].mean().values
    return center_sensors


def load_and_transform_scan(file_path, sensor_positions_df, center_sensors, sensor_id):
    # Load a scan, transform its coordinates into the global system, and return the transformed data.
    if os.path.exists(file_path):
        # Transform the CSV in a NumPy array
        data = np.genfromtxt(file_path, delimiter=',', skip_header=1, usecols=[5, 6, 7, 11])

        # Get the coordinates and object IDs
        xyz = data[:, :3]  # x, y, z coordinates
        object_ids = data[:, 3]  # Object IDs

        # Get the sensor information
        sensor_info = sensor_positions_df.iloc[sensor_id]
        if not sensor_info.empty:
            origin_sensor = sensor_info[
                ['x', 'y', 'z']].values.flatten()

            angles = np.radians(sensor_info[['x_rotation', 'y_rotation', 'z_rotation']].astype(float).values.flatten())

            R = rotation_matrix_from_euler_angles(angles)

            # Transform the coordinates
            transformed_xyz = transform_coordinates(xyz, origin_sensor, center_sensors, R)

            # Return the transformed data
            return transformed_xyz, object_ids

    return None
