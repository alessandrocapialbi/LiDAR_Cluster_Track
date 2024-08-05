import os
import open3d as o3d
from data_loader import load_point_clouds_from_sensors, load_trajectories
from ground_filter import filter_ground
from sensor_selection import select_sensors
from point_cloud_merger import merge_point_clouds
from clustering import clustering_and_bounding_boxes


def main():

    point_cloud_directory = "..\\..\\sensors\\"

    # Sensor to use
    selected_indices = select_sensors(5)

    # Load the point clouds from the selected sensors
    point_clouds = []
    for i in selected_indices:
        filenames = load_point_clouds_from_sensors(point_cloud_directory, selected_indices[i], range(20, 31))
        point_clouds.extend([o3d.io.read_point_cloud(filename) for filename in filenames])


    # Filtra il terreno dalle nuvole di punti
    z_threshold = -1.5
    filtered_point_clouds = [filter_ground(pcd, z_threshold) for pcd in point_clouds]

    # Converte le nuvole di punti filtrate in Open3D PointCloud objects
    o3d_point_clouds = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd)) for pcd in filtered_point_clouds]

    # Unisci le nuvole di punti filtrate
    merged_pcd = merge_point_clouds(o3d_point_clouds)

    # Esegui il clustering e crea bounding box
    clusters, bounding_boxes = clustering_and_bounding_boxes(merged_pcd)

    # Visualizza i risultati
    o3d.visualization.draw_geometries([merged_pcd] + bounding_boxes)


if __name__ == "__main__":
    main()
