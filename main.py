import os
import open3d as o3d
from data_loader import load_point_clouds_from_sensors, load_trajectories
from ground_filter import filter_ground
from sensor_selection import select_sensors
from point_cloud_merger import merge_point_clouds
from clustering import clustering_and_bounding_boxes


def main():
    # Sensor to use
    selected_indices = select_sensors(5)

    z_threshold = -1.5
    filtered_point_clouds = filter_ground(z_threshold)

    o3d_point_clouds = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd)) for pcd in filtered_point_clouds]

    # CONVERSION FROM SENSOR TO WORLD COORDINATES

    merged_pcd = merge_point_clouds(o3d_point_clouds)

    clusters, bounding_boxes = clustering_and_bounding_boxes(merged_pcd)

    o3d.visualization.draw_geometries([merged_pcd] + bounding_boxes)


if __name__ == "__main__":
    main()
