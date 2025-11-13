import os
import numpy as np
import pandas as pd


def load_point_clouds_from_sensors(directory, sensor_ids, scan_number):
    """
    Load point cloud CSV files from multiple sensors for a specific scan.

    Args:
        directory (str): Path to the folder containing the sensor CSV files.
        sensor_ids (list of int): List of sensor IDs to load.
        scan_number (int): Scan index number to load (used in filename).

    Returns:
        list of str: List of file paths for the existing sensor CSV files corresponding to the scan.

    Notes:
        - Filenames are expected in the format: 'sensor_<sensor_id>_<scan_number>.csv'
        - Only existing files are returned; missing files are skipped.
    """
    filenames = []
    for sensor_id in sensor_ids:
        filename = f'sensor_{sensor_id}_{scan_number:02d}.csv'
        file_path = os.path.join(directory, filename)
        if os.path.exists(file_path):
            filenames.append(file_path)
    return filenames


def load_file(file_path):
    """
    Load a CSV file into a pandas DataFrame.

    Args:
        file_path (str): Path to the CSV file.

    Returns:
        pd.DataFrame: DataFrame containing the CSV data.

    Notes:
        - Typically used to load sensor positions or metadata.
    """
    df = pd.read_csv(file_path)
    return df
