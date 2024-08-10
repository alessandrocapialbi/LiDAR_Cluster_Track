import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN


def dbscan_clustering(pcd, eps=0.5, min_points=10):
    points = np.asarray(pcd.points)

    clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points)

    labels = clustering.labels_

    # Set the labels
    unique_labels = set(labels)

    # Create the clusters
    clusters = [points[labels == label] for label in unique_labels if label != -1]

    return clusters


def create_bounding_boxes(clusters):
    bounding_boxes = []
    for cluster in clusters:
        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))
        bounding_boxes.append(bbox)
    return bounding_boxes
