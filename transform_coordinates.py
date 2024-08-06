import os
import numpy as np
import pandas as pd
from data_loader import load_sensor_positions


def transform_coordinates(xyz, origin_sensor, center_sensors):
    # Transform the coordinates of the point cloud
    origin_global = origin_sensor - center_sensors
    transformed_xyz = xyz + origin_global
    return transformed_xyz



