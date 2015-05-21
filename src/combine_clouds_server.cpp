#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <transparent_object_reconstruction/CombineClouds.h>
#include <transparent_object_reconstruction/CloudReset.h>
#include <transparent_object_reconstruction/common_typedefs.h>

LabelCloudPtr combined_cloud;
uint32_t current_label;
ros::Publisher pub;
tf::TransformListener* listener;
std_msgs::Header combined_cloud_header;

bool
reset (transparent_object_reconstruction::CloudReset::Request &req,
    transparent_object_reconstruction::CloudReset::Response &res)
{
  combined_cloud->points.clear (); // reset combined point cloud
  combined_cloud->width = combined_cloud->points.size ();
  current_label = 0; // reset labels
  ROS_INFO ("Cleared combined cloud and resetted labels");
  return true;
}


bool
combine (transparent_object_reconstruction::CombineClouds::Request &req,
    transparent_object_reconstruction::CombineClouds::Response &res)
{
  // convert to pcl
  pcl::PCLPointCloud2 tmp_cloud;
  pcl_conversions::moveToPCL (req.input_cloud, tmp_cloud);
  CloudPtr input (new Cloud);
  pcl::fromPCLPointCloud2 (tmp_cloud, *input);

  // check if empty combined_cloud
  if (combined_cloud->points.size () == 0)
  {
    pcl_conversions::toPCL (req.input_cloud.header, combined_cloud->header);
    // use frame_id and time stamp of first cloud as reference
    combined_cloud_header = req.input_cloud.header;
    ROS_DEBUG ("Set tf frame of combined_cloud to %s", combined_cloud_header.frame_id.c_str ());
  }
  else
  {
    // check if new cloud is in the same tf frame as combined cloud
    if (req.input_cloud.header.frame_id.compare (combined_cloud_header.frame_id) != 0)
    {
      // retrieve transform and apply it to input cloud
      tf::StampedTransform transform;
      try
      {
        listener->waitForTransform (combined_cloud_header.frame_id,
            req.input_cloud.header.frame_id, req.input_cloud.header.stamp,
            ros::Duration (5.0));
        listener->lookupTransform (combined_cloud_header.frame_id, req.input_cloud.header.frame_id,
            req.input_cloud.header.stamp, transform);
        pcl_ros::transformPointCloud (*input, *input, transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR ("Error looking up transform: %s; can't add point cloud to combined cloud",
            ex.what());
        return false;
      }
    }
  }

  combined_cloud->points.reserve (combined_cloud->points.size () + input->points.size ());
  LabelPoint lp;
  lp.label = current_label;
  // add input cloud to combined_cloud with an unused label
  Cloud::VectorType::const_iterator p_it = input->points.begin ();
  while (p_it != input->points.end ())
  {
    lp.x = p_it->x;
    lp.y = p_it->y;
    lp.z = p_it->z;
    combined_cloud->points.push_back (lp);
    p_it++;
  }
  combined_cloud->width = combined_cloud->points.size ();

  // convert combined cloud to std_msgs::PointCloud2 for response
  pcl::toPCLPointCloud2 (*combined_cloud, tmp_cloud);
  pcl_conversions::fromPCL (tmp_cloud, res.combined_cloud);
  pcl_conversions::fromPCL (combined_cloud->header, res.combined_cloud.header);
  
  // modify label value for next input cloud
  current_label++;
  ROS_INFO ("Input cloud contained %lu points, combined cloud contains %lu points with %u different labels",
      input->points.size (), combined_cloud->points.size (), current_label);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "combine_clouds_server");
  ros::NodeHandle n;

  listener = new tf::TransformListener;

  combined_cloud.reset (new LabelCloud);
  combined_cloud->height = 1;
  current_label = 0;

  ros::ServiceServer service = n.advertiseService ("combine_clouds", combine);
  ros::ServiceServer clear_service = n.advertiseService ("cloud_reset", reset);
  ROS_INFO("Ready to combine pointclouds");

  pub = n.advertise<LabelCloud> ("/combined_cloud", 1);

  // loop to continously publish the combined point cloud
  ros::Rate loop_rate (1.0);
  while (ros::ok ())
  {
    pub.publish (*combined_cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  return 0;
}
