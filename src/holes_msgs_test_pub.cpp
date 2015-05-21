#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <limits>

#include <transparent_object_reconstruction/Holes.h>

ros::Publisher holes_pub;

visualization_msgs::Marker hole_marker;

transparent_object_reconstruction::Holes holes;

void
cloud_cb (const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  // check if the current retrieved hole is already inside the hole collection
  std::vector<sensor_msgs::PointCloud2>::iterator hole_it;
  hole_it = holes.convex_hulls.begin ();
  while (hole_it != holes.convex_hulls.end ())
  {
    if (hole_it->header.frame_id.compare (cloud->header.frame_id) == 0)
    {
      break;
    }
    hole_it++;
  }
  if (hole_it != holes.convex_hulls.end ())
  {
    (*hole_it) = *cloud;
  }
  else
  {
    ROS_INFO ("Added a new convex hull containing %u points in frame %s",
        cloud->width * cloud->height, cloud->header.frame_id.c_str ());
    holes.convex_hulls.push_back (*cloud);
  }
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "holes_msgs_test_pub");
  ros::NodeHandle n_handle;
  
  ros::Subscriber sub = n_handle.subscribe ("/hole", 1, cloud_cb);

  holes_pub = n_handle.advertise<transparent_object_reconstruction::Holes> ("/hole_convex_hulls", 0);

  ros::Rate loop_rate (10);

  while (ros::ok ())
  {
    holes_pub.publish (holes);
    loop_rate.sleep ();
    ros::spinOnce ();
  }

  return EXIT_SUCCESS;
}
