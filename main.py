import os
import numpy as np
import open3d as o3d

filename = "C:\\Users\\aless\\Desktop\\LiDARClusterTrack\\alessandrocapialbi\\LiDAR_Cluster_Track\\filtered_sensors_data\\sensor_3_20.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=1, usecols=[5, 6, 7])

# Crea un oggetto PointCloud
pcd = o3d.geometry.PointCloud()
print("Numero di punti: ", len(data))
pcd.points = o3d.utility.Vector3dVector(data)
pcd = pcd.voxel_down_sample(voxel_size=0.3)
print("Numero di punti dopo il down sampling: ", len(pcd.points))


# Visualizza la nuvola di punti
o3d.visualization.draw_geometries([pcd])



