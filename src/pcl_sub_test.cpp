#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>

#include <pcl/console/parse.h>

#include <iostream>
#include <limits>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> KDTree;
typedef KDTree::Ptr KdTreePtr;


void
cloud_cb (const Cloud::ConstPtr &cloud)
{
  ROS_INFO ("cloud with %lu points", cloud->points.size ());
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_sub_test");
  ros::NodeHandle n_handle;

  ros::Subscriber sub = n_handle.subscribe ("/shifted_hole_as_cloud",
      1, cloud_cb);


  ros::spin ();

  return EXIT_SUCCESS;
}
