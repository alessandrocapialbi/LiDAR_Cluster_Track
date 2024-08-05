import open3d as o3d


def merge_point_clouds(point_clouds):
    merged_pcd = o3d.geometry.PointCloud()
    for pcd in point_clouds:
        merged_pcd += pcd
    return merged_pcd
