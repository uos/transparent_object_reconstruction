#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <iomanip>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/Holes.h>
#include <transparent_object_reconstruction/tools.h>

ros::Publisher vis_pub;
ros::Publisher all_holes_vis_pub;

visualization_msgs::Marker marker;
visualization_msgs::MarkerArray all_holes_marker;

std::vector<size_t> msgs_change_indices;


void
hole_hull_cb (const transparent_object_reconstruction::Holes::ConstPtr &holes)
{
  ROS_DEBUG ("Retrieved a total of %lu convex hulls", holes->convex_hulls.size ());

  visualization_msgs::Marker clear_marker(marker);
  // prevent RViz warning about empty frame_id
  clear_marker.header.frame_id = "map";
  // DELETEALL is not officially around before jade, addressed it by value
  clear_marker.action = 3;
  vis_pub.publish (clear_marker);

  if (holes->convex_hulls.size () == 0)
    return;

  std::stringstream ss;
  geometry_msgs::Point center;
  all_holes_marker.markers.reserve (all_holes_marker.markers.size () + holes->convex_hulls.size ());

  float r,g,b;
  float h = 0.0f;
  float color_increment = 360.f
    / static_cast<float>(holes->convex_hulls.size ());

  for (size_t i = 0; i < holes->convex_hulls.size (); ++i)
  {
    // convert sensor_msgs::PointCloud2 to pcl::PointCloud
    CloudPtr hull_cloud (new Cloud);
    pcl::fromROSMsg (holes->convex_hulls[i], *hull_cloud);

    // set up header etc. for marker
    visualization_msgs::Marker tmp_marker (marker);
    tmp_marker.id = i;
    tmp_marker.header = holes->convex_hulls[i].header;
    // assign marker with rainbow color, dependent on number of markers in holes msg
    hsv2rgb (h, r, g, b);
    tmp_marker.color.r = r;
    tmp_marker.color.g = g;
    tmp_marker.color.b = b;
    h += color_increment;

    tmp_marker.points.reserve ((hull_cloud->points.size () - 2) * 3);
    geometry_msgs::Point fixed_point, tp1, tp2;

    if (hull_cloud->points.size () < 3)
    {
      ROS_WARN ("Retrieved 'convex hull' consisting of only %lu points, skipping",
          hull_cloud->points.size ());
      continue;
    }

    // pick first point of convex hull polygon to belong to each tesselation triangle
    Cloud::VectorType::const_iterator p_it = hull_cloud->points.begin ();
    size_t counter = 0;
    fixed_point.x = p_it->x;
    fixed_point.y = p_it->y;
    fixed_point.z = p_it->z;
    p_it++;
    tp1.x = p_it->x;
    tp1.y = p_it->y;
    tp1.z = p_it->z;
    p_it++;
    while (p_it != hull_cloud->points.end ())
    {
      tp2.x = p_it->x;
      tp2.y = p_it->y;
      tp2.z = p_it->z;

      tmp_marker.points.push_back (fixed_point);
      tmp_marker.points.push_back (tp1);
      tmp_marker.points.push_back (tp2);
      tp1 = tp2;
      p_it++;
    }
    all_holes_marker.markers.push_back (tmp_marker);
    vis_pub.publish (tmp_marker);
  }

  // TODO: do we need to manually delete the old markers?
  // store where the current hole regions end in marker array
  msgs_change_indices.push_back (all_holes_marker.markers.size ());
  // do recoloring of markers, so that the markers of each frame are colored in the same color
  h = 0.0f;
  color_increment = 360.f / static_cast<float>(msgs_change_indices.size ());
  std::vector<float> red;
  std::vector<float> green;
  std::vector<float> blue;
  red.reserve (all_holes_marker.markers.size ());
  green.reserve (all_holes_marker.markers.size ());
  blue.reserve (all_holes_marker.markers.size ());
  size_t start, end;
  start = 0;
  for (size_t i = 0; i < msgs_change_indices.size (); ++i)
  {
    // determine index range of holes from the same msg
    end = msgs_change_indices[i];
    // create color for all holes from the same message
    hsv2rgb (h, r, g, b);
    h += color_increment;
    for (size_t j = start; j < end; ++j)
    {
      red.push_back (r);
      green.push_back (g);
      blue.push_back (b);
    }
    start = end;
  }
  for (size_t i = 0; i < all_holes_marker.markers.size (); ++i)
  {
    all_holes_marker.markers[i].color.r = red[i];
    all_holes_marker.markers[i].color.g = green[i];
    all_holes_marker.markers[i].color.b = blue[i];
  }
  all_holes_vis_pub.publish (all_holes_marker);
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "hole_visualizer");
  ros::NodeHandle n_handle;

  vis_pub = n_handle.advertise<visualization_msgs::Marker> ("/curr_hole_visualization", 10);

  all_holes_vis_pub = n_handle.advertise<visualization_msgs::MarkerArray> ("/all_hole_visualization", 10);
  msgs_change_indices.clear ();

  // setup generic marker field
  marker.ns = "table_holes";
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  ros::Subscriber sub = n_handle.subscribe ("/table_holes", 1, hole_hull_cb);

  ros::spin (); 

  return EXIT_SUCCESS;
}
