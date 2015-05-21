#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <transparent_object_reconstruction/CombineClouds.h>
#include <transparent_object_reconstruction/common_typedefs.h>

#include <pcl/console/parse.h>

namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
  // check if a valid file was specified via command line
  if (!(pcl::console::find_argument (argc, argv, "-f") > 0))
  {
    std::cerr << "no input cloud specified (use argument '-f' and specify pcd file)" << std::endl;
    return EXIT_FAILURE;
  }

  std::string pcd_file_name;
  pcl::console::parse_argument (argc, argv, "-f", pcd_file_name);
  fs::path tmp_path (pcd_file_name);

  if (!fs::is_regular_file (tmp_path))
  {
    std::cerr << "Specified input file '" << tmp_path.string () << "' ";
    if (!fs::exists (tmp_path))
    {
      std::cerr << "does not exist.";
    }
    else
    {
      std::cerr << "apparently is not a regulat file but perhaps"
        << " s directory.";
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
        << "' is no .pcd file. Exiting." << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::string frame_id = "/cam";
  if (pcl::console::find_argument (argc, argv, "-frame") > 0)
  {
    pcl::console::parse_argument (argc, argv, "-frame", frame_id);
  }

  ros::init(argc, argv, "combine_clouds_client");
  
  ros::NodeHandle n_handle;
  ros::ServiceClient client = n_handle.serviceClient<transparent_object_reconstruction::CombineClouds>("combine_clouds");
  transparent_object_reconstruction::CombineClouds srv;
 
  pcl::PCDReader reader;
  CloudPtr input (new Cloud);
  if (reader.read<PointType> (pcd_file_name, *input) < 0)
  {
    ROS_ERROR ("Error reading specified pcd file '%s'.",
        pcd_file_name.c_str ());
    return EXIT_FAILURE;
  }
  ROS_INFO ("Succesfully read input pcd file containing %lu points",
      input->points.size ());

  pcl::PCLPointCloud2 tmp_cloud;
  pcl::toPCLPointCloud2 (*input, tmp_cloud);
  pcl_conversions::moveFromPCL (tmp_cloud, srv.request.input_cloud);
  srv.request.input_cloud.header.stamp = ros::Time::now ();
  srv.request.input_cloud.header.frame_id = frame_id;
  

  // convert srv.response.combined_cloud to pcl PointCloud
  if (client.call(srv))
  {
    LabelCloudPtr retrieved_cloud (new LabelCloud);
    pcl_conversions::toPCL (srv.response.combined_cloud, tmp_cloud);
    pcl::fromPCLPointCloud2 (tmp_cloud, *retrieved_cloud);

    ROS_INFO ("Received combined cloud with %lu points",
       retrieved_cloud->points.size ());
  }
  else
  {
    ROS_ERROR("Failed to call service for combining clouds");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

