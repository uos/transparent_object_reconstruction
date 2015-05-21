#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <iomanip>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/Holes.h>

ros::Publisher vis_pub;

visualization_msgs::Marker marker;

// we assume saturation and value to be 1.0f, since we want bright distinguishable colors :)
void
hsv2rgb (float h, float &r, float &g, float &b)
{
  float h_ = h / 60.0f;
  float x, c;
  c = 1.0f;
  x = c * (1.0f - fabs (fmod (h_, 2) - 1));

  if (h_ < .0f)
  {
    ROS_WARN ("hsv2rgb: angle must be positive, provided angle: %f",  h);
    r = g = b = 1.0f;
  }
  else if (0.0f <= h_ && h_ < 1.0f)
  {
    r = c;
    g = x;
    b = 0.0f;
  }
  else if (1.0f <= h_ && h_ < 2.0f)
  {
    r = x;
    g = c;
    b = 0.0f;
  }
  else if (2.0f <= h_ && h_ < 3.0f)
  {
    r = 0.0f;
    g = c;
    b = x;
  }
  else if (3.0f <= h_ && h_ < 4.0f)
  {
    r = 0.0f;
    g = x;
    b = c;
  }
  else if (4.0f <= h_ && h_ < 5.0f)
  {
    r = x;
    g = 0.0f;
    b = c;
  }
  else if (5.0f <= h_ && h_ < 6.0f)
  {
    r = c;
    g = 0.0f;
    b = x;
  }
  else
  {
    ROS_WARN ("Angle must be less than 360°, provided angle: %f", h);
    r = g = b = 1.0f;
  }
}

void
hole_hull_cb (const transparent_object_reconstruction::Holes::ConstPtr &holes)
{
  ROS_DEBUG ("Retrieved a total of %lu convex hulls", holes->convex_hulls.size ());

  std::stringstream ss;
  geometry_msgs::Point center;
  pcl::PCLPointCloud2 tmp_cloud;

  float r,g,b;
  float h = 0.0f;
  float color_increment = 360.f
    / static_cast<float>(holes->convex_hulls.size ());

  for (size_t i = 0; i < holes->convex_hulls.size (); ++i)
  {
    // convert pcl::PointCloud2 to PointCloud
    pcl_conversions::toPCL (holes->convex_hulls[i], tmp_cloud);
    CloudPtr hull_cloud (new Cloud);
    pcl::fromPCLPointCloud2 (tmp_cloud, *hull_cloud);

    // set up header etc. for marker
    ss.str ("");
    ss << "convex_hull_hole" << std::setw (3) << std::setfill ('0')
      << i;
    // TODO: automatically assign markers with rainbow colors
    visualization_msgs::Marker tmp_marker (marker);
    tmp_marker.ns = ss.str ();
    tmp_marker.id = i;
    tmp_marker.header = holes->convex_hulls[i].header;
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

    vis_pub.publish (tmp_marker);
  }
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "hole_visualizer");
  ros::NodeHandle n_handle;

  vis_pub = n_handle.advertise<visualization_msgs::Marker> ("hole_visualization", 0);

  // setup generic marker field
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

  ros::Subscriber sub = n_handle.subscribe ("/hole_convex_hulls", 1, hole_hull_cb);

  ros::spin (); 

  return EXIT_SUCCESS;
}
