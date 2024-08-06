# Reduce the number of points in the point cloud by downsampling it to help speed up the real-time processing.
def downsample_pcd(pcd, voxel_size):
    downsampled_data = pcd.voxel_down_sample(voxel_size=voxel_size)
    return downsampled_data
