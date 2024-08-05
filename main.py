import numpy as np
import open3d as o3d

filename = r"C:\\Users\\aless\\Desktop\\LiDARClusterTrack\\sensors\sensor_4_27.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=1, usecols=[5, 6, 7])

z_threshold = -1.5

z_column_index = 2

car_data = data[data[:, z_column_index] > z_threshold]

# Crea un oggetto PointCloud
pcd = o3d.geometry.PointCloud()
print("Numero di punti: ", len(car_data))
pcd.points = o3d.utility.Vector3dVector(car_data)
num_points_before = len(pcd.points)

# Visualizza la nuvola di punti
o3d.visualization.draw_geometries([pcd])
