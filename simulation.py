import open3d as o3d
import numpy as np

def visualize(vis, pcd_combined, bboxes_and_trajectories):
    # Clear previous geometries
    vis.clear_geometries()

    # Add new geometries
    vis.add_geometry(pcd_combined)
    for bbox in bboxes_and_trajectories:
        vis.add_geometry(bbox)

    # Update the visualization
    vis.poll_events()
    vis.update_renderer()


def update_visualization(vis, pcd_combined, bboxes_and_trajectories):
    # Update the visualization in the main thread
    visualize(vis, pcd_combined, bboxes_and_trajectories)


def create_cylinder_between_points(point1, point2, color, radius=0.05, resolution=50):
    # Convert points to numpy arrays
    point1 = np.asarray(point1)
    point2 = np.asarray(point2)

    # Calculate the direction and length between the two points
    direction = point2 - point1
    length = np.linalg.norm(direction)

    # Check that the points are not the same
    if length <= 0:
        length = 0.6

    # Create a cylinder with a height equal to the distance between the two points
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=resolution)

    # Calculate the rotation to align the cylinder with the vector between point1 and point2
    direction_unit_vector = direction / length  # Normalized direction vector
    z_unit_vector = np.array([0, 0, 1])  # The cylinder is aligned along the z-axis

    # Calculate the axis and angle for rotation
    axis = np.cross(z_unit_vector, direction_unit_vector)
    angle = np.arccos(np.dot(z_unit_vector, direction_unit_vector))

    # Rotate the cylinder to align with the direction vector
    cylinder.rotate(o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle), center=(0, 0, 0))

    # Translate the cylinder to the start point
    cylinder.translate(point1)

    # Set the color of the cylinder
    cylinder.paint_uniform_color(color)

    return cylinder

