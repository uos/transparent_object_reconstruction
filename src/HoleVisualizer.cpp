/*
 * Copyright (c) 2015-2017, Sven Albrecht
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

Copyright (c) 2017, Sven Albrecht
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.

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

visualization_msgs::Marker marker;


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

    if (tesselateConvexHull<ColorPoint> (hull_cloud, tmp_marker))
    {
      vis_pub.publish (tmp_marker);
    }
  }
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "hole_visualizer");
  ros::NodeHandle n_handle;

  vis_pub = n_handle.advertise<visualization_msgs::Marker> ("transObjRec/curr_hole_visualization", 10);

  // setup generic marker field
  marker.ns = "table_holes";
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
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

  ros::Subscriber sub = n_handle.subscribe ("table_holes", 1, hole_hull_cb);

  ros::spin (); 

  return EXIT_SUCCESS;
}
