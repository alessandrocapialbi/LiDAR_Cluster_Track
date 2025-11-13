import os
import numpy as np


def rotation_matrix_from_euler_angles(angles):
    """
    Compute a 3D rotation matrix from Euler angles (roll, pitch, yaw).

    Args:
        angles (list or np.ndarray): Euler angles [roll, pitch, yaw] in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    roll, pitch, yaw = angles

    # Rotation about the X-axis (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Rotation about the Y-axis (pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation about the Z-axis (yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation (applied in order Z → Y → X)
    R = R_z @ R_y @ R_x
    return R


def transform_coordinates(xyz, origin_sensor, center_sensors, R):
    """
    Transform a point cloud from the sensor's local coordinate system
    to the global coordinate system.

    Args:
        xyz (np.ndarray): Nx3 array of point coordinates in the sensor frame.
        origin_sensor (np.ndarray): Sensor origin [x, y, z] in global coordinates.
        center_sensors (np.ndarray): Centroid of all sensors [x, y, z].
        R (np.ndarray): 3x3 rotation matrix of the sensor.

    Returns:
        np.ndarray: Nx3 array of transformed global coordinates.
    """
    # Translate sensor origin relative to sensor network center
    origin_global = origin_sensor - center_sensors

    # Apply translation and rotation
    transformed_xyz = xyz + origin_global
    transformed_xyz = np.dot(transformed_xyz, R)
    return transformed_xyz


def calculate_sensors_centroid(sensor_positions_df):
    """
    Compute the geometric centroid of all sensors.

    Args:
        sensor_positions_df (pd.DataFrame): DataFrame with sensor positions ('x', 'y', 'z').

    Returns:
        np.ndarray: 3-element array representing the centroid [x, y, z].
    """
    center_sensors = sensor_positions_df[['x', 'y', 'z']].mean().values
    return center_sensors


def load_and_transform_scan(file_path, sensor_positions_df, center_sensors, sensor_id):
    """
    Load a LiDAR scan, apply the appropriate sensor transformation,
    and return the transformed coordinates with object IDs.

    Args:
        file_path (str): Path to the CSV file containing the LiDAR scan.
        sensor_positions_df (pd.DataFrame): DataFrame with sensor positions and rotations.
        center_sensors (np.ndarray): Global centroid of all sensors.
        sensor_id (int): ID (index) of the sensor to use for transformation.

    Returns:
        tuple[np.ndarray, np.ndarray] | None:
            - transformed_xyz: Nx3 array of transformed global coordinates.
            - object_ids: Nx1 array of object identifiers.
            Returns None if the file does not exist or data is invalid.
    """
    if os.path.exists(file_path):
        # Load the scan as a NumPy array
        data = np.genfromtxt(file_path, delimiter=',', skip_header=1, usecols=[5, 6, 7, 11])

        # Separate coordinates and object IDs
        xyz = data[:, :3]
        object_ids = data[:, 3]

        # Retrieve sensor metadata
        sensor_info = sensor_positions_df.iloc[sensor_id]
        if not sensor_info.empty:
            origin_sensor = sensor_info[['x', 'y', 'z']].values.flatten()
            angles = np.radians(sensor_info[['x_rotation', 'y_rotation', 'z_rotation']].astype(float).values.flatten())

            # Compute rotation matrix and apply transformation
            R = rotation_matrix_from_euler_angles(angles)
            transformed_xyz = transform_coordinates(xyz, origin_sensor, center_sensors, R)

            return transformed_xyz, object_ids

    return None
