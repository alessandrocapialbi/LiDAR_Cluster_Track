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


def update_visualization(vis, pcd_combined, bounding_boxes):
    # Update the visualization in the main thread
    visualize(vis, pcd_combined, bounding_boxes)
