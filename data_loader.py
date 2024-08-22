import os
import numpy as np
import pandas as pd


# Load the point clouds from the sensors one by one.
def load_point_clouds_from_sensors(directory, sensor_ids, scan_number):
    filenames = []
    for sensor_id in sensor_ids:
        filename = f'sensor_{sensor_id}_{scan_number:02d}.csv'
        file_path = os.path.join(directory, filename)
        if os.path.exists(file_path):
            filenames.append(file_path)
    return filenames


def load_file(file_path):
    # Load the sensor positions from the CSV file.
    df = pd.read_csv(file_path)
    return df
