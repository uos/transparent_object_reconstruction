# Message to hold information about the outlines of holes on a planar surface
# Each hole is represented as a sensor_msgs::PointCloud2 where the points
# denote the outline of the convex hull of the hole, i.e. the polyline
# connecting conescutive points (and the end with the start point) constitutes
# the convex hull of the the hole. Note that the holes will usually come from
# different frames, but these can be accessed via the header information
# provided by sensor_msgs::PointCloud2

# collection of detected holes
sensor_msgs/PointCloud2[] convex_hulls
