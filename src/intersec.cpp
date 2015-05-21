#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include<dynamic_reconfigure/server.h>
#include<transparent_object_reconstruction/IntersecConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <transparent_object_reconstruction/common_typedefs.h>

#include <iostream>
#include <limits>
#include <vector>
#include <set>

typedef pcl::octree::OctreePointCloud<LabelPoint> LabelOctree;
typedef pcl::octree::OctreeContainerPointIndices LeafContainer;

ros::Publisher pub;
ros::Publisher vis_pub;

visualization_msgs::Marker intersec_marker;
visualization_msgs::Marker non_intersec_marker;

float octree_resolution; // TODO: dyn reconfig
unsigned int min_voxel_points;
LabelOctree::Ptr octree;

bool
isLeafInIntersection (const LabelCloud &leaf_cloud,
    const std::set<uint32_t> &labels)
{
  std::map<uint32_t, size_t> leaf_labels;
  std::map<uint32_t, size_t>::iterator detected_label;
  std::set<uint32_t>::const_iterator label_it = labels.begin ();

  while (label_it != labels.end ())
  {
    leaf_labels.insert (std::pair<uint32_t, size_t> (*label_it, 0));
    label_it++;
  }

  LabelCloud::VectorType::const_iterator p_it;
  p_it = leaf_cloud.points.begin ();
  while (p_it != leaf_cloud.points.end ())
  {
    detected_label = leaf_labels.find (p_it->label);
    if (detected_label != leaf_labels.end ())
    {
      detected_label->second++; // increase counter for detected label
    }
    else
    {
      ROS_WARN ("isLeafInIntersection (): found label that was not in set of available labels");
    }
    p_it++;
  }

  bool missing_label = false;
  size_t min_points_per_label = 1; // TODO: change this value dynammically dependent on overal number of points in leaf?
  
  detected_label = leaf_labels.begin ();
  while (detected_label != leaf_labels.end ())
  {
    if (detected_label->second < min_points_per_label)
    {
      missing_label = true;
      return false;
    }
    detected_label++;
  }
  if (!missing_label)
    return true;
  return false;
}

void
dyn_cb (transparent_object_reconstruction::IntersecConfig &config, uint32_t level)
{
  if (fabs (config.octree_resolution - octree->getResolution ()) > 0.00005)
  {
    ROS_INFO ("Changed Octree Resolution from %lf to %lf", octree->getResolution (),
        config.octree_resolution);
    // adapt octree resolution
    octree->deleteTree ();
    octree->setResolution (config.octree_resolution);
    // adapt marker size
    intersec_marker.scale.x = config.octree_resolution;
    intersec_marker.scale.y = config.octree_resolution;
    intersec_marker.scale.z = config.octree_resolution;
    non_intersec_marker.scale.x = config.octree_resolution;
    non_intersec_marker.scale.y = config.octree_resolution;
    non_intersec_marker.scale.z = config.octree_resolution;
    octree_resolution = config.octree_resolution;
  }
  else
  {
    ROS_WARN ("Difference between current resolution (%lf) and desired resolution (%lf) too small.",
        octree->getResolution (), config.octree_resolution);
  }
  if (fabs (config.octree_vis_alpha - intersec_marker.color.a) >= 0.01)
  {
    ROS_INFO ("Changed alpha from %lf to %lf", intersec_marker.color.a, config.octree_vis_alpha);
    intersec_marker.color.a = config.octree_vis_alpha;
    non_intersec_marker.color.a = config.octree_vis_alpha;
  }
}

void
cloud_cb (const ConstLabelCloudPtr &cloud)
{
  ROS_DEBUG ("got a cloud with %lu points", cloud->points.size ());
  
  // create output variable
  LabelCloudPtr intersec_cloud (new LabelCloud);
  intersec_cloud->header = cloud->header;
  intersec_cloud->points.reserve (cloud->points.size ());

  // make sure that the octree doesn't contain any old stuff anymore...
  octree->deleteTree ();
  // add pointcloud to octree
  octree->setInputCloud (cloud);
  octree->addPointsFromInputCloud ();

  // determine the number of labels in cloud
  std::set<uint32_t> labels;
  LabelCloud::VectorType::const_iterator p_it;
  p_it = cloud->points.begin ();
  while (p_it != cloud->points.end ())
  {
    labels.insert (p_it->label);
    p_it++;
  }
  ROS_DEBUG ("Detected %lu labels", labels.size ());

  // TODO: could add some stats computation about data for fun
 
  // iterate over all leafs and check which leaves are filled with points from all labels
  LabelOctree::LeafNodeIterator leaf_it;
  leaf_it = octree->leaf_begin ();
  pcl::PointIndices::Ptr leaf_point_indices (new pcl::PointIndices);
  LabelCloudPtr leaf_cloud (new LabelCloud);
  pcl::ExtractIndices<LabelPoint> extract;
  extract.setInputCloud (cloud);

  // get nr occupied voxels
  // TODO: check if there is a more efficient way to do this
  LabelCloud::VectorType leaf_centers;
  octree->getOccupiedVoxelCenters (leaf_centers);

  // setup visualization marker
  std_msgs::Header tmp_header;
  pcl_conversions::fromPCL (intersec_cloud->header, tmp_header);
  // variables to extract and propagate leaf volumes
  geometry_msgs::Point voxel_center;
  Eigen::Vector3f min, max, center;

  // TODO: should we remove old message marker?
  // clear old contents
  intersec_marker.points.clear ();
  non_intersec_marker.points.clear ();
  intersec_marker.points.reserve (leaf_centers.size ());
  non_intersec_marker.points.reserve (leaf_centers.size ());

  // update marker header information
  intersec_marker.header = tmp_header;
  non_intersec_marker.header = tmp_header;

  size_t nr_leaves, filled_leaves, intersec_leaves;
  nr_leaves = filled_leaves = intersec_leaves = 0;
  while (leaf_it != octree->leaf_end ())
  {
    nr_leaves++;
    // remove old point indices
    leaf_point_indices->indices.clear ();

    // retreave container for the current leaf
    LeafContainer &container = leaf_it.getLeafContainer ();
    container.getPointIndices (leaf_point_indices->indices);
    
    // check if enough points in leaf
    if (leaf_point_indices->indices.size () > min_voxel_points)
    {
      filled_leaves++;
      extract.setIndices (leaf_point_indices);
      extract.filter (*leaf_cloud);

      // retrieve the center point of the current leaf
      octree->getVoxelBounds (leaf_it, min, max);
      center = min + max;
      center /= 2.0f;
      voxel_center.x = center[0];
      voxel_center.y = center[1];
      voxel_center.z = center[2];

      if (isLeafInIntersection (*leaf_cloud, labels))
      {
        intersec_marker.points.push_back (voxel_center);

        intersec_leaves++;
        p_it = leaf_cloud->points.begin ();
        while (p_it != leaf_cloud->points.end ())
        {
          intersec_cloud->points.push_back (*p_it++);
        }
      }
      else
      {
        non_intersec_marker.points.push_back (voxel_center);
      }
    }
    leaf_it++;
  }
  ROS_DEBUG ("Nr leaves: %lu,\t nr filled leaves: %lu,\t intersec_leaves: %lu,\t intersec_points: %lu",
      nr_leaves, filled_leaves, intersec_leaves, intersec_cloud->points.size ());
  
  intersec_cloud->width = intersec_cloud->points.size ();
  intersec_cloud->height = 1;

  // publish intersection cloud
  pub.publish (*intersec_cloud);

  ROS_INFO ("Published intersected cloud with %lu points", intersec_cloud->points.size ());

  // publish visualization marker
  vis_pub.publish (intersec_marker);
  vis_pub.publish (non_intersec_marker);
}


int
main (int argc, char **argv)
{
  ros::init (argc, argv, "intersec");
  ros::NodeHandle n_handle;

  octree_resolution = 0.005f;
  min_voxel_points = 10;

  octree.reset (new LabelOctree (octree_resolution));

  intersec_marker.ns = "intersec";
  intersec_marker.id = 0;
  intersec_marker.type = visualization_msgs::Marker::CUBE_LIST;
  intersec_marker.action = visualization_msgs::Marker::ADD;
  intersec_marker.pose.position.x = 0;
  intersec_marker.pose.position.y = 0;
  intersec_marker.pose.position.z = 0;
  intersec_marker.pose.orientation.x = 0.0;
  intersec_marker.pose.orientation.y = 0.0;
  intersec_marker.pose.orientation.z = 0.0;
  intersec_marker.pose.orientation.w = 1.0;
  intersec_marker.scale.x = octree_resolution;
  intersec_marker.scale.y = octree_resolution;
  intersec_marker.scale.z = octree_resolution;
  intersec_marker.color.a = 0.25;
  intersec_marker.color.r = 0.0;
  intersec_marker.color.g = 1.0;
  intersec_marker.color.b = 0.0;

  non_intersec_marker = visualization_msgs::Marker (intersec_marker);
  non_intersec_marker.ns = "non_intersec";
  non_intersec_marker.id = 1;
  non_intersec_marker.color.r = 1.0;
  non_intersec_marker.color.g = 0.0;

  pub = n_handle.advertise <LabelCloud> ("/intersection", 1);

  vis_pub = n_handle.advertise<visualization_msgs::Marker>( "octree_vis", 0 );

  // create and set up dynamic reconfigure server
  dynamic_reconfigure::Server<transparent_object_reconstruction::IntersecConfig> server;
  dynamic_reconfigure::Server<transparent_object_reconstruction::IntersecConfig>::CallbackType f;

  f = boost::bind (&dyn_cb, _1, _2);
  server.setCallback (f);

  ros::Subscriber sub = n_handle.subscribe ("/fused_cloud", 1, cloud_cb);
 
  ros::spin ();

  return EXIT_SUCCESS;
}
