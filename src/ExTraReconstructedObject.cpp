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

#include <bag_loop_check/bag_loop_check.hpp>

#include <map>
#include <iostream>
#include <limits>


typedef pcl::octree::OctreePointCloud<LabelPoint> LabelOctree;
typedef pcl::octree::OctreeContainerPointIndices LeafContainer;

class ExTraReconstructedObject
{
  public:
    ExTraReconstructedObject (float cluster_tolerance, size_t min_cluster_size, size_t max_cluster_size) :
      cluster_tolerance_ (cluster_tolerance),
      min_cluster_size_ (min_cluster_size),
      max_cluster_size_ (max_cluster_size)
    {
      param_handle_ = ros::NodeHandle ("~");

      // retrieve minimal ratio of detected labels or use default parameter
      param_handle_.param<int> ("angle_resolution", angle_resolution_, ANGLE_RESOLUTION);
      param_handle_.param<int> ("opening_angle", opening_angle_, OPENING_ANGLE);
      param_handle_.param<float> ("median_fraction", median_fraction_, MEDIAN_FRACTION);

      voxel_cloud_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/voxelized_intersection", 10, true);

      refined_voxel_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/refined_voxel", 10, true);
      refined_intersec_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/refined_intersec", 10, true);

      cluster_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/intersec_clusters", 10, true);

      all_hulls_vis_pub_ = nhandle_.advertise<visualization_msgs::MarkerArray> ("transObjRec/intersec_cluster_hulls", 10, true);

      result_pub_ = nhandle_.advertise<object_recognition_msgs::RecognizedObjectArray> ("transObjRec/trans_recon_results", 10, false);

      intersec_sub_ = nhandle_.subscribe ("transObjRec/intersection", 5, &ExTraReconstructedObject::intersec_cb, this);

      ROS_INFO ("created ExTraReconstructedObject and subscribed to topic");

      setUpVisMarker ();

      db_type = "{\"type\":\"empty\"}";
    };

    void intersec_cb (const LabelCloud::ConstPtr &cloud)
    {
      ROS_INFO ("received intersection cloud with %lu points", cloud->points.size ());

      static int call_counter = 0;

      // check for bag loop
      static bag_loop_check::BagLoopCheck bagloop;
      if (bagloop)
      {
        param_handle_.param<int> ("angle_resolution", angle_resolution_, ANGLE_RESOLUTION);
        param_handle_.param<int> ("opening_angle", opening_angle_, OPENING_ANGLE);
        param_handle_.param<float> ("median_fraction", median_fraction_, MEDIAN_FRACTION);
      }

      // clear old marker
      pcl_conversions::fromPCL (cloud->header, clear_marker_array_.markers.front ().header);
      all_hulls_vis_pub_.publish (clear_marker_array_);

      // reduce number of points that need to be clustered via voxel grid
      LabelCloudPtr leaf_center_cloud (new LabelCloud);
      float octree_resolution = 0.01f;  // TODO: make accessible parameter
      LabelOctree::Ptr octree (new LabelOctree (octree_resolution));
      octree->setInputCloud (cloud);
      octree->addPointsFromInputCloud ();
      // get dimensions of octree
      Eigen::Vector3d min_bbox, max_bbox;
      octree->getBoundingBox (min_bbox[0], min_bbox[1], min_bbox[2], max_bbox[0], max_bbox[1], max_bbox[2]);
      // use the occupied leaf centers for voxelization of the point cloud
      octree->getOccupiedVoxelCenters (leaf_center_cloud->points);
      // adapt dimensions of point cloud
      leaf_center_cloud->width = leaf_center_cloud->points.size ();
      leaf_center_cloud->height = 1;

      ROS_INFO ("created grid cloud with %lu total points", leaf_center_cloud->points.size ());
      leaf_center_cloud->header = cloud->header; // explicitly copy header information from incoming point cloud
      voxel_cloud_pub_.publish (*leaf_center_cloud);
      
      // TODO: in theory a 3D region growing could also be done here and proof quicker (iff cloud was voxelized)
      // do Euclidean clustering on the voxelized cloud
      pcl::search::KdTree<LabelPoint>::Ptr tree (new pcl::search::KdTree<LabelPoint>);
      tree->setInputCloud (leaf_center_cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<LabelPoint> ec;
      ec.setClusterTolerance (cluster_tolerance_);
      ec.setMinClusterSize (min_cluster_size_);
      ec.setMaxClusterSize (max_cluster_size_);
      ec.setSearchMethod (tree);
      ec.setInputCloud (leaf_center_cloud);
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
        extract_object_indices.setInputCloud (leaf_center_cloud);
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

      int id_x, id_y, id_z;

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

        // ====== refinement filter of the extracted clusters =====

        // collect all labels in the current cluster
        std::set<uint32_t> all_labels_in_cluster;

        // let's just try the complete intersection with all available cluster labels here...
        LabelCloudPtr refined_intersection (new LabelCloud);
        LabelCloudPtr refined_voxel_centers (new LabelCloud);

        // create extraction object
        pcl::ExtractIndices<LabelPoint> extract;
        extract.setInputCloud (cloud);
        std::vector<LabelCloud> leaf_clouds;
        leaf_clouds.reserve (output[i]->points.size ());
        std::vector<size_t> leaf_label_numbers;
        leaf_label_numbers.reserve (output[i]->points.size ());

        pcl::PointIndices::Ptr leaf_point_indices (new pcl::PointIndices);
        // iterate over all leaf centers in the current cluster
        LabelCloud::VectorType::const_iterator leaf_center_it = output[i]->points.begin ();
        size_t total_cluster_points = 0;

        // ===== bin visualization =====
        std::stringstream img_ss;
        img_ss << "bit_arrays_frame" << std::setw (3) << std::setfill ('0') << call_counter
          <<"_cluster" << std::setw (3) << std::setfill ('0') << i << ".pbm";
        std::ofstream img (img_ss.str ().c_str ());
        img << "P1" << "\n#Visualization of bit arrays in cluster " << i << "\n";
        Eigen::Vector3d approx_cluster_center = Eigen::Vector3d::Zero ();
        // map to hold pairs of center point indices with their bin array (as a string), ordered according to the
        // number of bin entries
        std::multimap<size_t, std::pair<size_t, std::string> > img_map;
        // ===== bin visualization =====

        size_t center_index = 0;
        while (leaf_center_it != output[i]->points.end ())
        {
          // retrieve the octree indices of the current center
          getOctreeIndices<LabelPoint> (min_bbox, *leaf_center_it, octree_resolution, id_x, id_y, id_z);
          // retrieve the leaf container associated with the indices
          if (octree->existLeaf (id_x, id_y, id_z))
          {
            LeafContainer* curr_container = octree->findLeaf (id_x, id_y, id_z);
            if (curr_container != 0)
            {
              LabelCloudPtr leaf_cloud (new LabelCloud);
              // remove old leaves
              leaf_point_indices->indices.clear ();
              // retrieve point indices of current leaf
              curr_container->getPointIndices (leaf_point_indices->indices);
              // retrieve the points from the container
              extract.setIndices (leaf_point_indices);
              extract.filter (*leaf_cloud);

              std::set<uint32_t> leaf_labels;
              LabelCloud::VectorType::const_iterator leaf_point_it = leaf_cloud->points.begin ();
              while (leaf_point_it != leaf_cloud->points.end ())
              {
                leaf_labels.insert (leaf_point_it->label);
                leaf_point_it++;
              }

              // ===== bin visualization =====
              std::vector<uint8_t> view_bin_marker (angle_resolution_, 0);
              LabelCloud::VectorType::const_iterator marker_it = leaf_cloud->points.begin ();
              while (marker_it != leaf_cloud->points.end ())
              {
                for (int k = -opening_angle_; k <= opening_angle_; ++k)
                {
                  view_bin_marker[(marker_it->label + k + angle_resolution_) % angle_resolution_] = 1;
                }
                marker_it++;
              }
              std::stringstream img_ss;
              size_t img_map_counter = 0;
              for (size_t k = 0; k < view_bin_marker.size (); ++k)
              {
                img_ss << view_bin_marker[k] << " ";
                img_map_counter += view_bin_marker[k];
              }
              img_ss << std::endl;
              img_map.insert (std::pair<size_t, std::pair<size_t, std::string> > (img_map_counter,
                    std::pair<size_t, std::string> (center_index, img_ss.str ())));

              approx_cluster_center[0] += leaf_center_it->x;
              approx_cluster_center[1] += leaf_center_it->y;
              approx_cluster_center[2] += leaf_center_it->z;
              // ===== bin visualization =====

              // store points of current leaf
              leaf_clouds.push_back (*leaf_cloud);
              // store nr of labels of current leaf
              leaf_label_numbers.push_back (leaf_labels.size ());
              // add leaf labels to collection of cluster labels
              all_labels_in_cluster.insert (leaf_labels.begin (), leaf_labels.end ());
              total_cluster_points += leaf_cloud->points.size ();
            }
            else
            {
              ROS_WARN ("ExTraReconstructedObject: leaf exists, but doesn't provide valid container");
            }
          }
          else
          {
            ROS_WARN ("ExTraReconstructedObject: specified indices %i %i %i don't refer to an existing leaf.",
                id_x, id_y, id_z);
          }
          leaf_center_it++;
          center_index++;
        }

        // ===== bin visualization =====
        img << "# cluster contained labels at the following positions: ";
        std::set<uint32_t>::const_iterator label_it = all_labels_in_cluster.begin ();
        while (label_it != all_labels_in_cluster.end ())
        {
          img << *label_it++ << " ";
        }
        img << std::endl;
        img << "# approximated cluster center: " << approx_cluster_center[0] << ", "
          << approx_cluster_center[1] << ", " << approx_cluster_center[2] << std::endl;
        img << angle_resolution_ << output[i]->points.size () << std::endl;

        std::multimap<size_t, std::pair<size_t, std::string> >::const_iterator map_it = img_map.begin ();
        while (map_it != img_map.end ())
        {
          img << map_it->second.second;
          map_it++;
        }
        img.flush ();
        img.close ();
        // ===== bin visualization =====

        // get the number of bin-entries of the median
        map_it = img_map.begin ();
        for (size_t j = 0; j < img_map.size () / 2; ++j)
        {
          map_it++;
        }
        size_t median = map_it->first;
        size_t median_bin_threshold = static_cast<size_t> (median_fraction_ * median);
        // iterate over all leaves that are to be ignored
        map_it = img_map.begin ();
        while (map_it->first < median_bin_threshold)
        {
          map_it++;
        }
        // store all other leaves as intersection cloud and refined voxel centers
        while (map_it != img_map. end ())
        {
          refined_voxel_centers->points.push_back (output[i]->points[map_it->second.first]);
          refined_intersection->points.insert (refined_intersection->points.end (),
              leaf_clouds[map_it->second.first].points.begin (),
              leaf_clouds[map_it->second.first].points.end ());
          map_it++;
        }
        // TODO: try mean based post-filtering instead of median

        // set header and adapt the dimensions of the refined clouds
        refined_intersection->header = output[i]->header;
        refined_intersection->width = refined_intersection->points.size ();
        refined_intersection->height = 1;
        refined_voxel_centers->header = output[i]->header;
        refined_voxel_centers->width = refined_voxel_centers->points.size ();
        refined_voxel_centers->height = 1;

        if (i == 0)
        {
          refined_voxel_pub_.publish (refined_voxel_centers);
          refined_intersec_pub_.publish (refined_intersection);
        }

        // ====== refinement filter of the extracted clusters =====


        // compute Convex hull for current cluster
        LabelCloudPtr convex_hull (new LabelCloud);
        std::vector<pcl::Vertices> polygons;
        pcl::ConvexHull<LabelPoint> c_hull;
        c_hull.setInputCloud (refined_intersection);
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
        shape_msgs::Mesh curr_mesh;
        tesselate3DConvexHull<LabelPoint> (convex_hull, polygons, curr_hull_marker, cog, curr_mesh);
        pcl_conversions::fromPCL (cloud->header, curr_hull_marker.header);

        // add current marker to marker array
        all_hulls.markers.push_back (curr_hull_marker);

        // create recognized obj
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
        pcl::toROSMsg (*leaf_center_cloud, pc2);
        o.point_clouds.push_back (pc2);
        // add the mesh to recognized obj
        o.bounding_mesh = curr_mesh;

        // TODO: should covariance be set to identity matrix or remain 0-matrix?

        transparent_recon_objs->objects.push_back (o);
      }
      
      // publish the convex hulls of all clusters
      all_hulls_vis_pub_.publish (all_hulls);

      result_pub_.publish (transparent_recon_objs);

      call_counter++;

      ROS_INFO ("finished callback, published %lu clusters with a total of %lu points",
          output.size (), total_points);

    };

  protected:
    ros::NodeHandle nhandle_;
    ros::NodeHandle param_handle_;
    ros::Subscriber intersec_sub_;

    ros::Publisher voxel_cloud_pub_;
    ros::Publisher all_hulls_vis_pub_;
    ros::Publisher cluster_pub_;
    ros::Publisher result_pub_;
    ros::Publisher refined_intersec_pub_;
    ros::Publisher refined_voxel_pub_;

    visualization_msgs::Marker hull_marker_;
    visualization_msgs::MarkerArray clear_marker_array_;

    std::string db_type;

    // TODO: set these values (via constructor?)
    float cluster_tolerance_;
    size_t min_cluster_size_;
    size_t max_cluster_size_;
    // parameters for intersection computation in the individual clusters
    int angle_resolution_;
    int opening_angle_;
    float median_fraction_;

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
