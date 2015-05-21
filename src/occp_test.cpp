#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/console/parse.h>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/hole_region.h>
#include <transparent_object_reconstruction/occupancy_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>

typedef pcl::ModelCoefficients Model;
typedef Model::Ptr ModelPtr;
typedef std::vector<ModelPtr> ModelVector;

namespace fs = boost::filesystem;

const double PLANE_INLIER_THRESHOLD = 0.005; // 1cm
// threshold for checking for another plane in the scene
const double MIN_PLANE_CHECK_INLIER_RATIO = 0.25;
const size_t MIN_NR_PLANE_INLIERS = 15000;
// threshold to decide when a grid cell is on a plane
const float ON_PLANE_THRESHOLD = 0.02f;
// threshold for minimal size of connected grid cells for a HoleRegion
const size_t MIN_HOLE_REGION_SIZE = 10;

/**
 * Method to calculate the affine transformations to move various planes
 * into the x-y plane.
 *
 * @param[in] plane_coeff_vector A vector containing plane coefficients
 * @param[out] transformation_vector A vector containing transformations
 *    to align each plane in 'plane_coeff_vector' with the x-y-plane
 **/
void
calcPlaneTransformations (const ModelVector &plane_coeff_vector,
    std::vector<Eigen::Affine3f> &transformation_vector)
{
  // clear contents of transformation_vector
  transformation_vector.clear ();

  Eigen::Vector3f origin;

  for (size_t i = 0; i < plane_coeff_vector.size (); ++i)
  {
    // retrieve the normal of the plane
    Eigen::Vector3f plane_normal (plane_coeff_vector[i]->values[0],
        plane_coeff_vector[i]->values[1],
        plane_coeff_vector[i]->values[2]);
    // check if this is already sufficiently close to the normal of
    // the x-y-plane - in this case we can use the identity as the
    // transformation
    double angle = pcl::getAngle3D (Eigen::Vector4f (plane_normal[0],
          plane_normal[1], plane_normal[2], 0.0f),
        Eigen::Vector4f::Unit (4,2));
    // check if there is a smaller equivalent angle (0°=360°;15°=345°)
    if (fabs (angle - 2.0 * M_PI) < angle)
    {
      angle = fabs (angle - 2.0 * M_PI);
    }
    // check if normal basically corresponds to z-axis (less than 1° angle)
    if (angle < pcl::deg2rad (1.0))
    {
      // create point on plane: x and y can be chosen arbitrarily
      origin = Eigen::Vector3f (0.0f, 0.0f, -plane_coeff_vector[i]->values[3]);

      // define y-axis
      Eigen::Vector3f y_axis (0.0f, 1.0f, 0.0f);

      Eigen::Affine3f transformation;
      pcl::getTransformationFromTwoUnitVectorsAndOrigin (y_axis,
          plane_normal, origin, transformation);
      transformation_vector.push_back (transformation);
    }
    else
    {
      // create a vector that's orthogonal to the normal vector
      // therefore create a vector whose dot product with the normal
      // vector is 0
      float x, y, z;
      float tmp_float = -plane_coeff_vector[i]->values[3];
      // normal[2] is larger than 0
      if (fabs (plane_normal[2]) > std::numeric_limits<float>::epsilon ())
      {
        x = y = 1.0f; // chosen arbitrarily
        // z = x * plane_normal[0] + y * plane_normal[1], with x and y being 1.0
        z = plane_normal[0] + plane_normal[1];
        z /= -plane_normal[2];

        // create point on plane
        origin = Eigen::Vector3f (0.0f, 0.0f,
            tmp_float / plane_coeff_vector[i]->values[2]);
      }
      // normal[2] is 0, normal[1] is larger than 0
      else if (fabs (plane_normal[1]) > std::numeric_limits<float>::epsilon ())
      {
        x = z = 1.0f; // chosen arbitrarily
        y = plane_normal[0];
        y /= -plane_normal[1];

        // create point on plane
        origin = Eigen::Vector3f (0.0f,
            tmp_float / plane_coeff_vector[i]->values[1],
            0.0f);
      }
      // normal[1] and normal[2] are 0
      else
      {
        x = 0.0f;
        y = z = 1.0f; // chosen arbitrarily

        // create point on plane
        origin = Eigen::Vector3f (tmp_float /
            plane_coeff_vector[i]->values[0], 0.0f, 0.0f);
      }
      Eigen::Vector3f ortho_to_normal (x, y, z);
      ortho_to_normal.normalize ();

      // assert to check for correct calculation of ortho_to_normal
      float dot_prod = plane_normal.dot (ortho_to_normal);
      assert (fabs (dot_prod) < 2 * std::numeric_limits<float>::epsilon ());

      // calculate the transformation of the plane into the x-y plane
      Eigen::Affine3f transformation;
      pcl::getTransformationFromTwoUnitVectorsAndOrigin (ortho_to_normal,
          plane_normal, origin, transformation);

      transformation_vector.push_back (transformation);
    }
  }
}

int
main(int argc, char **argv)
{
  ros::init (argc, argv, "occp_test");
  ros::NodeHandle n_handle;


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

  CloudPtr cloud (new Cloud);
  CloudPtr remaining_points (new Cloud);
  CloudPtr plane_points (new Cloud);

  pcl::PCDReader reader;
  if (reader.read<PointType> (pcd_file_name, *cloud) < 0)
  {
    ROS_ERROR ("Error reading specified pcd file '%s'.",
        pcd_file_name.c_str ());
    return EXIT_FAILURE;
  }

  ROS_INFO ("Succesfully read input pcd file containing %lu points",
      cloud->points.size ());

  // for testing just extract the first plane...
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr plane_coefficients;

  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (PLANE_INLIER_THRESHOLD);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *plane_coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR ("Couldn't extract plane from input file '%s'; exiting",
        pcd_file_name.c_str ());
    return EXIT_FAILURE;
  }

  pcl::ExtractIndices<PointType> extract_object_indices;
  extract_object_indices.setNegative (false);
  extract_object_indices.setInputCloud (cloud);
  extract_object_indices.setIndices (inliers);
  extract_object_indices.filter (*plane_points);

  // set the thresholds for all OccupancyGrids
  OccupancyGrid::setMinHoleRegionSize (MIN_HOLE_REGION_SIZE);
  OccupancyGrid::setOnPlaneThreshold (ON_PLANE_THRESHOLD);

  while (ros::ok ())
  {
    ROS_INFO ("bla");
    ros::spinOnce ();
  }

  return EXIT_SUCCESS;
}
