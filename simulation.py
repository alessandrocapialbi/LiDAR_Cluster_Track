import open3d as o3d
import numpy as np

def visualize(vis, pcd_combined, bboxes_and_trajectories):
    """
    Update the Open3D visualizer with a new point cloud and geometric objects.

    This function clears the previous geometries from the visualization window,
    adds the new combined point cloud and all bounding boxes or trajectory
    geometries, and refreshes the renderer.

    Parameters
    ----------
    vis : o3d.visualization.Visualizer
        The Open3D visualizer instance.
    pcd_combined : o3d.geometry.PointCloud
        The combined point cloud to display.
    bboxes_and_trajectories : list
        A list of Open3D geometries, such as bounding boxes and trajectory cylinders.

    Returns
    -------
    None
    """
    vis.clear_geometries()
    vis.add_geometry(pcd_combined)
    for bbox in bboxes_and_trajectories:
        vis.add_geometry(bbox)
    vis.poll_events()
    vis.update_renderer()


def update_visualization(vis, pcd_combined, bboxes_and_trajectories):
    """
    Wrapper function to update the Open3D visualization in the main thread.

    This function calls `visualize()` to refresh the visualization window
    with the new point cloud and geometries.

    Parameters
    ----------
    vis : o3d.visualization.Visualizer
        The Open3D visualizer instance.
    pcd_combined : o3d.geometry.PointCloud
        The combined point cloud to display.
    bboxes_and_trajectories : list
        A list of Open3D geometries, such as bounding boxes and trajectory cylinders.

    Returns
    -------
    None
    """
    visualize(vis, pcd_combined, bboxes_and_trajectories)


def create_cylinder_between_points(point1, point2, color, radius=0.05, resolution=50):
    """
    Create a 3D cylinder mesh connecting two points in space.

    The cylinder is used to visually represent a segment of a trajectory or
    a connection between two spatial points. It is aligned along the vector
    connecting `point1` and `point2`, translated to start at `point1`, and
    painted with a uniform color.

    Parameters
    ----------
    point1 : array-like of shape (3,)
        The starting point of the cylinder.
    point2 : array-like of shape (3,)
        The ending point of the cylinder.
    color : array-like of shape (3,)
        RGB color of the cylinder (values in range [0, 1]).
    radius : float, optional
        Radius of the cylinder. Default is 0.05.
    resolution : int, optional
        Number of segments used to create the cylinder mesh. Default is 50.

    Returns
    -------
    o3d.geometry.TriangleMesh
        The cylinder mesh connecting the two points.
    """
    point1 = np.asarray(point1)
    point2 = np.asarray(point2)

    direction = point2 - point1
    length = np.linalg.norm(direction)
    if length <= 0:
        length = 0.6

    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=resolution)

    direction_unit_vector = direction / length
    z_unit_vector = np.array([0, 0, 1])

    axis = np.cross(z_unit_vector, direction_unit_vector)
    angle = np.arccos(np.dot(z_unit_vector, direction_unit_vector))

    cylinder.rotate(o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle), center=(0, 0, 0))
    cylinder.translate(point1)
    cylinder.paint_uniform_color(color)

    return cylinder
