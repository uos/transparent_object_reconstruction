# Message to contain information about the (voxelized) volume of (potential)
# transparent objects.

# The centers of the occupied voxelized volumes
sensor_msgs/PointCloud2 voxel_centers

# The viewpoint intervals associated with the voxel centers
transparent_object_reconstruction/VoxelViewPointIntervals[] voxel_intervals

# The point labels contained in voxel
transparent_object_reconstruction/VoxelLabels[] voxel_labels
