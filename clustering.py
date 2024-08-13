import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics import silhouette_score
from sklearn.metrics import davies_bouldin_score
from sklearn.metrics import calinski_harabasz_score
import matplotlib.pyplot as plt
import os

path = os.path.dirname(os.path.abspath(__file__))
output_file_path = os.path.join(path, 'transformed/color.csv')


def dbscan_clustering(pcd, eps=None, min_points=6, plot_k_distance=False):
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
        print(np.argmax(np.diff(k_distances_sorted)))
        # Suggest an eps based on the point of maximum slope
        eps = k_distances_sorted[np.argmax(np.diff(k_distances_sorted))]
        print(f"Suggested eps: {eps}")

    # Run DBSCAN with the determined eps value
    clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points)

    labels = clustering.labels_

    # Create clusters
    unique_labels = set(labels)
    clusters = [points[labels == label] for label in unique_labels if label != -1]

    silhouette_avg = silhouette_score(points, labels)
    db_index = davies_bouldin_score(points, labels)
    ch_index = calinski_harabasz_score(points, labels)
    print(f"Silhouette Score: {silhouette_avg}\n")
    print(f"Davies-Bouldin Index: {db_index}")
    print(f"Calinski-Harabasz Index: {ch_index}")

    return clusters, labels


def create_bounding_boxes(clusters):
    bounding_boxes = []
    for cluster in clusters:
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
        bounding_boxes.append(bbox)
    return bounding_boxes
