




import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import os

path = os.path.dirname(os.path.abspath(__file__))
output_file_path = os.path.join(path, 'transformed/color.csv')


def dbscan_clustering(pcd, eps=0.5, min_points=10):
    points = np.asarray(pcd.points)

    clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points)

    labels = clustering.labels_

    # Set the labels
    unique_labels = set(labels)

    # Create the clusters
    clusters = [points[labels == label] for label in unique_labels if label != -1]

    return clusters


def create_bounding_boxes(clusters, color, line_width=0.02):
    bounding_boxes = []
    for cluster in clusters:
        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster))

        # Set the color of the bounding box
        bbox.color = color

        # Create the lines of the bounding box
        line_set = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(bbox)
        line_set.paint_uniform_color(color)

        # Create the coordinate frame to thickness the lines
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=line_width, origin=[0, 0, 0])
        lines = []
        for line in line_set.lines:
            for i in range(4):
                line_points = line_set.points[np.array(line)]
                segment = o3d.geometry.LineSet.create_from_triangle_mesh(mesh_frame)
                segment.translate(line_points[0])
                lines.append(segment)

        bounding_boxes.append(bbox)
        bounding_boxes.extend(lines)
    return bounding_boxes
