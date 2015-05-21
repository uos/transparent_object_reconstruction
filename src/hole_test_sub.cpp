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
  
ros::Publisher pub;
Eigen::Vector3f origin;

void
cloud_cb (const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  pcl::PCLPointCloud2 tmp_cloud;
  CloudPtr hole_cloud (new Cloud);
  pcl_conversions::toPCL (*cloud, tmp_cloud);
  pcl::fromPCLPointCloud2 (tmp_cloud, *hole_cloud);

  pcl_conversions::toPCL (cloud->header, hole_cloud->header);

  ROS_INFO ("retrieved cloud with %lu points", hole_cloud->points.size ());

  Cloud::VectorType::iterator p_it;
  p_it = hole_cloud->points.begin ();
  while (p_it != hole_cloud->points.end ())
  {
    p_it->z += 0.5f;
    p_it->r = 255;
    p_it->g = 0;
    p_it->b = 0;
    p_it++;
  }

  pub.publish (*hole_cloud);

}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "hole_test_sub");
  ros::NodeHandle n_handle;

  ros::Subscriber sub = n_handle.subscribe ("/hole", 1, cloud_cb);

  pub = n_handle.advertise<Cloud> ("/shifted_hole_as_cloud", 1);

  ros::spin ();

  return EXIT_SUCCESS;
}
