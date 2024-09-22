import open3d as o3d
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics import silhouette_score
from sklearn.metrics import davies_bouldin_score
from sklearn.metrics import calinski_harabasz_score
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os


path = os.path.dirname(os.path.abspath(__file__))
point_clouds_file_path = os.path.join(path, 'output/point_clouds.csv')
bounding_boxes_file_path = os.path.join(path, 'output/bounding_boxes.csv')


def dbscan_clustering(pcd, scan_number, eps=None, min_points=10, plot_k_distance=False, append = True):
    points = np.asarray(pcd.points)
    if eps is None:
        # Calculate the K-distance graph to determine eps
        neighbors = NearestNeighbors(n_neighbors=min_points).fit(points)
        distances, _ = neighbors.kneighbors(points)
        k_distances_sorted = np.sort(distances, axis=0)[:, -1]

        if plot_k_distance:
            # Plot the K-distance graph
            plt.figure(figsize=(10, 6))
            plt.plot(k_distances_sorted)
            plt.xlabel('Points sorted (in ascending order)')
            plt.ylabel(f'Distance to the {min_points}-th nearest neighbor')
            plt.title(f'K-distance Graph (min_samples = {min_points})')
            plt.grid(True)
            plt.show()

        # Suggest an eps based on the point of maximum slope
        eps = k_distances_sorted[np.argmax(np.diff(k_distances_sorted))]
        print(f"Suggested eps: {eps}")

    # Run DBSCAN with the determined eps value
    clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points)

    labels = clustering.labels_

    # Create clusters
    unique_labels = set(labels)
    clusters = [points[labels == label] for label in unique_labels if label != -1]

    # Save labeled points to CSV
    scan_numbers = np.full((points.shape[0], 1), scan_number)
    labeled_points = np.hstack((scan_numbers, points, labels.reshape(-1, 1)))
    df = pd.DataFrame(labeled_points, columns=['scan', 'x', 'y', 'z', 'label'])
    mode = 'a' if append else 'w'
    header = not append
    df.to_csv(point_clouds_file_path, mode=mode, header=header, index=False)

    silhouette_avg = silhouette_score(points, labels)
    db_index = davies_bouldin_score(points, labels)
    ch_index = calinski_harabasz_score(points, labels)
    print(f"Silhouette Score: {silhouette_avg}")
    print(f"Davies-Bouldin Index: {db_index}")
    print(f"Calinski-Harabasz Index: {ch_index}\n")

    return clusters, labels


def create_bounding_boxes(clusters, scan_number, append = True):
    bounding_boxes = []
    bbox_centroids = []
    bbox_coordinates = []
    for cluster in clusters:
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
        bbox.color = [0.0, 0.0, 0.0]
        bbox_centroids.append(bbox.get_center())
        bounding_boxes.append(bbox)
        bbox_coordinates.append(bbox.get_box_points())


    # Save bounding box coordinates to CSV
    bbox_data = []
    for i, bbox in enumerate(bbox_coordinates):
        for point in bbox:
            bbox_data.append([scan_number, i] + point.tolist())

    df_bbox = pd.DataFrame(bbox_data, columns=['scan', 'bbox_id', 'x', 'y', 'z'])
    mode = 'a' if append else 'w'
    header = not append
    df_bbox.to_csv(bounding_boxes_file_path, mode=mode, header=header, index=False)

    return bounding_boxes, bbox_centroids


def associate_ids_to_bboxes(centroids, object_ids, transformed_xyz):
    """
    Associates object IDs to bounding box centroids based on the minimum distance.
    """
    # Initialize a list for associations
    bbox_ids = [None] * len(centroids)

    for i, centroid in enumerate(centroids):
        # Calculate the distance between the centroid and all points
        distances = np.linalg.norm(transformed_xyz - np.array(centroid), axis=1)
        # Find the ID of the closest point
        closest_point_index = np.argmin(distances)
        bbox_ids[i] = object_ids[closest_point_index]

    return bbox_ids

