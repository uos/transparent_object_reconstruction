#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <dynamic_reconfigure/server.h>
#include <transparent_object_reconstruction/CreateRaysConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>

#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <transparent_object_reconstruction/common_typedefs.h>

#include <iostream>
#include <limits>

typedef pcl::search::KdTree<PointType> KDTree;
typedef KDTree::Ptr KdTreePtr;
  
ros::Publisher pub;
Eigen::Vector3f origin;
double sample_dist;
double std_dev_factor;
bool random_noise;
unsigned int call_counter = 0;

boost::mt19937 gen (42u); // seed generator
boost::normal_distribution<float> norm_distr;
boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > *rnd_number = 0;

// dynamic reconfigure callback - is used when the sample distance is changed
void
dyn_cb (transparent_object_reconstruction::CreateRaysConfig &config, uint32_t level)
{
  // level isn't used so far
  // change sample_dist;
  if (config.sample_dist != sample_dist)
    sample_dist = config.sample_dist;
  if (random_noise != config.random_noise)
    random_noise = config.random_noise;
  if (std_dev_factor != config.std_dev_factor)
  {
    std_dev_factor = config.std_dev_factor;
  }

  norm_distr = boost::normal_distribution<float> (sample_dist, sample_dist * std_dev_factor);

  ROS_INFO ("dyn_cb: %lf, %s, %lf", sample_dist,
      random_noise? "True" : "False", std_dev_factor);

  if (rnd_number != 0)
  {
    delete rnd_number;
  }
  rnd_number = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > (gen, norm_distr);
}

// callback for a pcl::PointCloud<PointType>
void
cloud_cb (const Cloud::ConstPtr &cloud)
{
  // create cloud to publish and compy header information
  CloudPtr ray_cloud (new Cloud);
  ray_cloud->header = cloud->header;
  

  size_t max_sample_points;
  double ray_length;
  unsigned int nr_steps;
  Eigen::Vector3f direction, voxel_vec, ray;
  Eigen::Vector3f step_vec, curr_sample;
  PointType min, max, sample_point;
  
  // get upper bound estimation of the number of points contained in ray cloud
  pcl::getMinMax3D (*cloud, min, max);
  // create point that is farthest away from origin of bbox
  Eigen::Vector3f upper_bound (
      (fabs (min.x) > fabs (max.x)) ? min.x : max.x,
      (fabs (min.y) > fabs (max.y)) ? min.y : max.y,
      (fabs (min.z) > fabs (max.z)) ? min.z : max.z);
  double max_dist = std::sqrt (upper_bound.dot (upper_bound));
  max_sample_points = cloud->points.size () *
    std::ceil (max_dist / sample_dist);

  ray_cloud->points.reserve (max_sample_points);

  float used_sample_dist;

  // create sampled ray for every 'hole point'
  Cloud::VectorType::const_iterator hole_it;
  hole_it = cloud->points.begin ();
  while (hole_it != cloud->points.end ())
  {
    // determine length and nr samples
    ray = Eigen::Vector3f (hole_it->x, hole_it->y, hole_it->z);
    ray_length = sqrt (ray.dot (ray));  // note: only works if cam at (0,0,0)

    if (random_noise)
    {
      do
      {
        used_sample_dist = (*rnd_number) (); // next normal distributed number around sample_dist
      }
      while (used_sample_dist <= 0 || used_sample_dist > ray_length);
    }
    else
    {
      used_sample_dist = sample_dist;
    }

    nr_steps = std::floor (ray_length / used_sample_dist);
    step_vec = ray.normalized ();
    step_vec *= -used_sample_dist;
    curr_sample = ray;

    sample_point = *hole_it;
    for (size_t i = 0; i < nr_steps; ++i)
    {
      sample_point.x = curr_sample[0];
      sample_point.y = curr_sample[1];
      sample_point.z = curr_sample[2];
      ray_cloud->points.push_back (sample_point);
      curr_sample += step_vec;
    }
    hole_it++;
  }
  ray_cloud->height = 1;
  ray_cloud->width = ray_cloud->points.size ();

  ROS_INFO ("Published ray_cloud with %lu sample points; max_sample_points: %lu",
      ray_cloud->points.size (), max_sample_points);

  // publish as PointCloud
  pub.publish (*ray_cloud);
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "create_rays");
  ros::NodeHandle n_handle;

  pub = n_handle.advertise<Cloud> ("/occlusion_frustum", 1);

  // create and set up dynamic reconfigure server
  dynamic_reconfigure::Server<transparent_object_reconstruction::CreateRaysConfig> server;
  dynamic_reconfigure::Server<transparent_object_reconstruction::CreateRaysConfig>::CallbackType f;

  f = boost::bind(&dyn_cb, _1, _2);
  server.setCallback(f);

  // subscribe to a topic publishing a pcl::PointCloud<PointType>
  ros::Subscriber sub = n_handle.subscribe ("/hole", 1, cloud_cb);

  ros::spin ();

  return EXIT_SUCCESS;
}
