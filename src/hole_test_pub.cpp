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

namespace fs = boost::filesystem;

void
usage (int arg, char **argv)
{
  std::cout << "usage:\nrosrun transparent_object_reconstruction hole_test_pub"
    <<" -f pcd_file" << std::endl;
}

int
main (int argc, char **argv)
{
  if (!(pcl::console::find_argument (argc, argv, "-f") > 0))
  {
    std::cerr << "no input cloud specified" << std::endl;
    usage (argc, argv);
    return EXIT_FAILURE;
  }
  std::string hole_file_name;
  pcl::console::parse_argument (argc, argv, "-f", hole_file_name);
  fs::path tmp_path (hole_file_name);
  if (!fs::is_regular_file (tmp_path))
  {
    std::cerr << "Specified input file '" << tmp_path.string () << "' ";
    if (!fs::exists (tmp_path))
    {
      std::cerr << "does not exist.";
    }
    else
    {
      std::cerr << "apparently is not a regular file but perhaps"
        << " a directory.";
    }
    std::cerr << " Exiting." << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    std::string extension_string = tmp_path.extension ().string ();
    boost::algorithm::to_lower (extension_string);
    if (extension_string.compare (".pcd") != 0)
    {
      std::cerr << "Specified input file '" << tmp_path.string ()
        << "' is no .pcd file. Exiting" << std::endl;
      return EXIT_FAILURE;
    }
  }
  std::string cloud_frame = "/cam";
  if (pcl::console::find_argument (argc, argv, "--frame") > 0)
  {
    std::string tmp_string;
    pcl::console::parse_argument (argc, argv, "--frame", tmp_string);
    if (tmp_string.substr (0,1).compare ("/") != 0)
    {
      ROS_WARN ("frame %s should start with '/'. Ignoring input and publishing in default-fram %s",
          tmp_string.c_str (), cloud_frame.c_str ());
    }
    else
    {
      cloud_frame = tmp_string;
    }
  }


  ros::init (argc, argv, "hole_test_pub");
  ros::NodeHandle n_handle;

  pub = n_handle.advertise<sensor_msgs::PointCloud2> ("/hole", 1);
  ros::Rate loop_rate (0.25);

  sensor_msgs::PointCloud2 pc2;
  // TODO: fill pc2 with content and set header frame id

  pcl::PCDReader reader;

  // read in a hole_cloud
  CloudPtr input (new Cloud);
  if (reader.read<PointType> (hole_file_name, *input) < 0)
  {
    ROS_ERROR ("Error reading specified pcd file '%s'.",
        hole_file_name.c_str ());
    return EXIT_FAILURE;
  }
  ROS_INFO ("Succesfully read input pcd file containing %lu points",
      input->points.size ());
  pcl::PCLPointCloud2 tmp_cloud;
  pcl::toPCLPointCloud2 (*input, tmp_cloud);
  pcl_conversions::fromPCL (tmp_cloud, pc2);
  pc2.header.frame_id = cloud_frame;

  while (ros::ok ())
  {
    pc2.header.stamp = ros::Time::now ();  
    pub.publish (pc2);
    ROS_INFO ("published hole cloud on topic /hole");
    loop_rate.sleep ();
    ros::spinOnce ();
  }
  return EXIT_SUCCESS;
}
