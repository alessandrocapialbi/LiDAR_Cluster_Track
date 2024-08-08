import os
import numpy as np


def transform_coordinates(xyz, origin_sensor, center_sensors):
    # Transform the coordinates of the point cloud
    origin_global = origin_sensor - center_sensors
    transformed_xyz = xyz + origin_global
    return transformed_xyz


def calculate_sensors_centroid(sensor_positions_df):
    # Calculate the centroid of the sensors
    center_sensors = sensor_positions_df[['x', 'y', 'z']].mean().values
    return center_sensors


def load_and_transform_scan(file_path, sensor_positions_df, center_sensors, sensor_id):
    # Load a scan, transform its coordinates into the global system, and return the transformed data.
    """''''''
    print(sensor_id)
    print("\n")
    ''''''"""
    if os.path.exists(file_path):
        # Transform the CSV in a NumPy array
        xyz = np.genfromtxt(file_path, delimiter=',', skip_header=1, usecols=[5, 6, 7])  # x, y, z are in cols 6, 7, 8

        # Get the sensor information
        sensor_info = sensor_positions_df.iloc[sensor_id]
        if not sensor_info.empty:
            origin_sensor = sensor_info[
                ['x', 'y', 'z']].values.flatten()

            # Transform the coordinates
            transformed_xyz = transform_coordinates(xyz, origin_sensor, center_sensors)

            # Return the transformed data
            return transformed_xyz

    return None
