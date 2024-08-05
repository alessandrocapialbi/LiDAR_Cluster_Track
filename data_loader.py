import numpy as np


def load_point_clouds(file_paths):
    point_clouds = []
    for file_path in file_paths:
        data = np.genfromtxt(file_path, delimiter=',', skip_header=1, usecols=[5, 6, 7])
        point_clouds.append(data)
    return point_clouds


def load_trajectories(file_path):
    return np.loadtxt(file_path, delimiter=',', skiprows=1)
