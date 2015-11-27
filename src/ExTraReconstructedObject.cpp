#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <dynamic_reconfigure/server.h>
#include <transparent_object_reconstruction/CreateRaysConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>

#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/tools.h>

#include <iostream>
#include <limits>


class ExTraReconstructedObject
{
  public:
    ExTraReconstructedObject (float cluster_tolerance, size_t min_cluster_size, size_t max_cluster_size) :
      cluster_tolerance_ (cluster_tolerance),
      min_cluster_size_ (min_cluster_size),
      max_cluster_size_ (max_cluster_size)
    {
      voxel_cloud_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/voxelized_intersection", 10, true);

      cluster_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/intersec_clusters", 10, true);

      all_hulls_vis_pub_ = nhandle_.advertise<visualization_msgs::MarkerArray> ("transObjRec/intersec_cluster_hulls", 10, true);

      result_pub_ = nhandle_.advertise<object_recognition_msgs::RecognizedObjectArray> ("transObjRec/trans_recon_results", 10, false);

      intersec_sub_ = nhandle_.subscribe ("transObjRec/intersection", 1, &ExTraReconstructedObject::intersec_cb, this);

      ROS_INFO ("created ExTraReconstructedObject and subscribed to topic");

      setUpVisMarker ();

      db_type = "{\"type\":\"empty\"}";
    };

    void intersec_cb (const LabelCloud::ConstPtr &cloud)
    {
      ROS_INFO ("received intersection cloud with %lu points", cloud->points.size ());

      // clear old marker
      pcl_conversions::fromPCL (cloud->header, clear_marker_array_.markers.front ().header);
      all_hulls_vis_pub_.publish (clear_marker_array_);

      // reduce number of points that need to be clustered via voxel grid
      LabelCloudPtr grid_cloud (new LabelCloud);
      pcl::VoxelGrid<LabelPoint> v_grid;
      v_grid.setInputCloud (cloud);
      v_grid.setLeafSize (0.01f, 0.01f, 0.01f);
      v_grid.setMinimumPointsNumberPerVoxel (1);
      v_grid.filter (*grid_cloud);

      ROS_INFO ("created grid cloud with %lu total points", grid_cloud->points.size ());
      grid_cloud->header = cloud->header; // explicitly copy header information from incoming point cloud
      voxel_cloud_pub_.publish (*grid_cloud);
      
      // TODO: in theory a 3D region growing could also be done here and proof quicker (iff cloud was voxelized)
      // do Euclidean clustering on the voxelized cloud
      pcl::search::KdTree<LabelPoint>::Ptr tree (new pcl::search::KdTree<LabelPoint>);
      tree->setInputCloud (grid_cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<LabelPoint> ec;
      ec.setClusterTolerance (cluster_tolerance_);
      ec.setMinClusterSize (min_cluster_size_);
      ec.setMaxClusterSize (max_cluster_size_);
      ec.setSearchMethod (tree);
      ec.setInputCloud (grid_cloud);
      ec.extract (cluster_indices);

      // retrieve the actual clusters from the indices
      std::vector<LabelCloudPtr> output;
      // clear the output vector
      output.clear ();
      // reserve space for each cluster
      output.reserve (cluster_indices.size ());

      // loop to store every cluster as a separate point cloud in output vector
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
          it != cluster_indices.end (); ++it)
      {
        pcl::ExtractIndices<LabelPoint> extract_object_indices;
        extract_object_indices.setInputCloud (grid_cloud);
        extract_object_indices.setIndices (pcl::PointIndices::Ptr (new pcl::PointIndices(*it)));
        LabelCloudPtr tmp (new LabelCloud);
        extract_object_indices.filter (*tmp);
        output.push_back (tmp);
      }
  
      // assign distinct colors and label to each cluster
      float h, r, g, b, color_increment;
      uint8_t red, green, blue;
      color_increment = 360.0f / static_cast<float> (output.size ());
      h = 0.0f;
      size_t total_points = 0;

      // prepare markers for convex hulls
      visualization_msgs::MarkerArray all_hulls;
      all_hulls.markers.reserve (output.size ());

      // prepare recognized object array
      object_recognition_msgs::RecognizedObjectArray::Ptr transparent_recon_objs = boost::make_shared<object_recognition_msgs::RecognizedObjectArray> ();
      // set header - should correspond with the header from the cloud
      pcl_conversions::fromPCL (cloud->header, transparent_recon_objs->header);
      transparent_recon_objs->objects.reserve (output.size ());

      for (size_t i = 0; i < output.size (); ++i)
      {
        hsv2rgb (h, r, g, b);
        red = static_cast<uint8_t> (r * 255);
        green = static_cast<uint8_t> (g * 255);
        blue = static_cast<uint8_t> (b * 255);
        LabelCloud::VectorType::iterator p_it = output[i]->points.begin ();
        while (p_it != output[i]->points.end ())
        {
          p_it->r = red;
          p_it->g = green;
          p_it->b = blue;
          p_it->label = i;
          p_it++;
        }
        output[i]->header = cloud->header; // explicitly copy header information
        cluster_pub_.publish (*output[i]);
        h += color_increment;
        total_points += output[i]->points.size ();

        // compute Convex hull for current cluster
        LabelCloudPtr convex_hull (new LabelCloud);
        std::vector<pcl::Vertices> polygons;
        pcl::ConvexHull<LabelPoint> c_hull;
        c_hull.setInputCloud (output[i]);
        c_hull.setDimension (3);
        c_hull.reconstruct (*convex_hull, polygons);

        // create marker for convex hull
        std::stringstream ss;
        ss << "convex_hull_intersec_cluster" << std::setw (2) << std::setfill ('0') << i;
        visualization_msgs::Marker curr_hull_marker (hull_marker_);
        curr_hull_marker.id = i;
        curr_hull_marker.ns = ss.str ();
        curr_hull_marker.color.r = r;
        curr_hull_marker.color.g = g;
        curr_hull_marker.color.b = b;

        Eigen::Vector3f cog;
        tesselate3DConvexHull<LabelPoint> (convex_hull, polygons, curr_hull_marker, cog);
        pcl_conversions::fromPCL (cloud->header, curr_hull_marker.header);

        // add current marker to marker array
        all_hulls.markers.push_back (curr_hull_marker);

        // create riecongnized obj
        ss.str ("");
        ss << "trans_obj" << std::setw (2) << std::setfill ('0') << i;

        object_recognition_msgs::RecognizedObject o;
        o.type.db = db_type;
        o.type.key = ss.str ();
        o.confidence = 1.0f;
        o.header = transparent_recon_objs->header;
        o.pose.header = transparent_recon_objs->header;
        // use center of gravity as reference point
        o.pose.pose.pose.position.x = cog[0];
        o.pose.pose.pose.position.y = cog[1];
        o.pose.pose.pose.position.z = cog[2];
        // use orientation from tabletop
        o.pose.pose.pose.orientation.x = 0.0f;
        o.pose.pose.pose.orientation.y = 0.0f;
        o.pose.pose.pose.orientation.z = 0.0f;
        o.pose.pose.pose.orientation.w = 1.0f;
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg (*grid_cloud, pc2);
        o.point_clouds.push_back (pc2);
        // TODO: should the mesh also be inserted into the recognizedObj?

        // TODO: should covariance be set to identity matrix or remain 0-matrix?

        transparent_recon_objs->objects.push_back (o);
      }
      
      // publish the convex hulls of all clusters
      all_hulls_vis_pub_.publish (all_hulls);

      result_pub_.publish (transparent_recon_objs);

      ROS_INFO ("finished callback, published %lu clusters with a total of %lu points",
          output.size (), total_points);

    };

  protected:
    ros::NodeHandle nhandle_;
    ros::Subscriber intersec_sub_;

    ros::Publisher voxel_cloud_pub_;
    ros::Publisher all_hulls_vis_pub_;
    ros::Publisher cluster_pub_;
    ros::Publisher result_pub_;

    visualization_msgs::Marker hull_marker_;
    visualization_msgs::MarkerArray clear_marker_array_;

    std::string db_type;

    // TODO: set these values (via constructor?)
    float cluster_tolerance_;
    size_t min_cluster_size_;
    size_t max_cluster_size_;

    void setUpVisMarker (void)
    {
      // setup basic marker for convex hull
      hull_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
      hull_marker_.pose.position.x = 0;
      hull_marker_.pose.position.y = 0;
      hull_marker_.pose.position.z = 0;
      hull_marker_.pose.orientation.x = 0.0;
      hull_marker_.pose.orientation.y = 0.0;
      hull_marker_.pose.orientation.z = 0.0;
      hull_marker_.pose.orientation.w = 1.0;
      hull_marker_.scale.x = 1.0;
      hull_marker_.scale.y = 1.0;
      hull_marker_.scale.z = 1.0;
      hull_marker_.color.a = 0.5;
      hull_marker_.color.r = 0.0;
      hull_marker_.color.g = 0.0;
      hull_marker_.color.b = 0.0;

      visualization_msgs::Marker clear_marker (hull_marker_);
      // DELETEALL is not officially around before jade, addressed it by value
      clear_marker.action = 3;
      clear_marker_array_.markers.push_back (clear_marker);
    };
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "ExTraReconstructedObject");

  ExTraReconstructedObject e (0.05f, 50, 500000);

  ros::NodeHandle n_handle;
  
  ros::spin ();

  return EXIT_SUCCESS;
}
