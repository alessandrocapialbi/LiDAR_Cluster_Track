import os
import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics import (
    silhouette_score,
    davies_bouldin_score,
    calinski_harabasz_score
)

# Define output file paths
path = os.path.dirname(os.path.abspath(__file__))
point_clouds_file_path = os.path.join(path, 'output/point_clouds.csv')
bounding_boxes_file_path = os.path.join(path, 'output/bounding_boxes.csv')

# Remove old output files if they exist
if os.path.exists(point_clouds_file_path) and os.path.exists(bounding_boxes_file_path):
    os.remove(point_clouds_file_path)
    os.remove(bounding_boxes_file_path)


def dbscan_clustering(pcd, scan_number, eps=None, min_points=10, plot_k_distance=False):
    """
    Perform DBSCAN clustering on a given 3D point cloud and save results.

    Args:
        pcd (o3d.geometry.PointCloud): Input Open3D point cloud.
        scan_number (int): Scan number identifier.
        eps (float, optional): The epsilon parameter for DBSCAN. If None, it's estimated automatically.
        min_points (int): Minimum number of points per cluster (DBSCAN min_samples).
        plot_k_distance (bool): Whether to plot the K-distance graph for eps estimation.

    Returns:
        tuple:
            - clusters (list of np.ndarray): List of clusters, each containing point coordinates.
            - labels (np.ndarray): Array of cluster labels for each point.

    Notes:
        - When eps is not provided, it is estimated based on the point of maximum curvature
          in the K-distance graph.
        - Clustering quality metrics (Silhouette, Davies-Bouldin, Calinski-Harabasz) are printed.
        - The clustered points are saved in `output/point_clouds.csv`.
    """
    points = np.asarray(pcd.points)

    # Automatically estimate eps using K-distance if not provided
    if eps is None:
        neighbors = NearestNeighbors(n_neighbors=min_points).fit(points)
        distances, _ = neighbors.kneighbors(points)
        k_distances_sorted = np.sort(distances, axis=0)[:, -1]

        if plot_k_distance:
            plt.figure(figsize=(10, 6))
            plt.plot(k_distances_sorted)
            plt.xlabel('Points (sorted ascending)')
            plt.ylabel(f'Distance to {min_points}-th nearest neighbor')
            plt.title(f'K-distance Graph (min_samples={min_points})')
            plt.grid(True)
            plt.show()

        # Estimate eps where the slope is maximum
        eps = k_distances_sorted[np.argmax(np.diff(k_distances_sorted))]
        print(f"Suggested eps: {eps}")

    # Run DBSCAN clustering
    clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points)
    labels = clustering.labels_

    # Extract valid clusters (exclude noise: label = -1)
    unique_labels = set(labels)
    clusters = [points[labels == label] for label in unique_labels if label != -1]

    # Save labeled points
    scan_numbers = np.full((points.shape[0], 1), scan_number)
    labeled_points = np.hstack((scan_numbers, points, labels.reshape(-1, 1)))
    df = pd.DataFrame(labeled_points, columns=['scan', 'x', 'y', 'z', 'label'])
    df.to_csv(point_clouds_file_path, mode='a', header=not os.path.exists(point_clouds_file_path), index=False)

    # Compute clustering quality metrics
    silhouette_avg = silhouette_score(points, labels)
    db_index = davies_bouldin_score(points, labels)
    ch_index = calinski_harabasz_score(points, labels)
    print(f"Silhouette Score: {silhouette_avg:.3f}")
    print(f"Davies-Bouldin Index: {db_index:.3f}")
    print(f"Calinski-Harabasz Index: {ch_index:.3f}\n")

    return clusters, labels


def create_bounding_boxes(clusters, scan_number):
    """
    Create oriented bounding boxes (OBBs) from clusters and save their coordinates.

    Args:
        clusters (list of np.ndarray): List of point clusters.
        scan_number (int): Scan number identifier.

    Returns:
        tuple:
            - bounding_boxes (list of o3d.geometry.OrientedBoundingBox): List of bounding box objects.
            - bbox_centroids (list of np.ndarray): List of bounding box centers (centroids).

    Notes:
        - Bounding box coordinates are saved in `output/bounding_boxes.csv`.
        - Each bounding box is assigned a unique ID per scan.
    """
    bounding_boxes = []
    bbox_centroids = []
    bbox_coordinates = []

    for cluster in clusters:
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
        bbox.color = [0.0, 0.0, 0.0]
        bounding_boxes.append(bbox)
        bbox_centroids.append(bbox.get_center())
        bbox_coordinates.append(bbox.get_box_points())

    # Save bounding box corner coordinates
    bbox_data = []
    for i, bbox in enumerate(bbox_coordinates):
        for point in bbox:
            bbox_data.append([scan_number, i] + point.tolist())

    df_bbox = pd.DataFrame(bbox_data, columns=['scan', 'bbox_id', 'x', 'y', 'z'])
    df_bbox.to_csv(bounding_boxes_file_path, mode='a', header=not os.path.exists(bounding_boxes_file_path), index=False)

    return bounding_boxes, bbox_centroids


def associate_ids_to_bboxes(centroids, object_ids, transformed_xyz):
    """
    Associate detected object IDs to bounding boxes based on centroid proximity.

    Args:
        centroids (list of np.ndarray): List of bounding box centroids.
        object_ids (np.ndarray): Array of object IDs corresponding to points.
        transformed_xyz (np.ndarray): Transformed point coordinates.

    Returns:
        list: Object IDs associated with each bounding box.

    Notes:
        - The nearest point (Euclidean distance) determines the associated object ID.
        - Useful for tracking or labeling detected objects.
    """
    bbox_ids = [None] * len(centroids)

    for i, centroid in enumerate(centroids):
        distances = np.linalg.norm(transformed_xyz - np.array(centroid), axis=1)
        closest_point_index = np.argmin(distances)
        bbox_ids[i] = object_ids[closest_point_index]

    return bbox_ids
