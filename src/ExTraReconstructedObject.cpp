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

// boost interval stuff for alternative implementation of viewpoint overlap based reconstruction...
#include <boost/icl/interval_set.hpp>
#include <boost/icl/discrete_interval.hpp>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/tools.h>

#include <transparent_object_reconstruction/VoxelizedTransObjInfo.h>

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

      // define all publishers
      refined_intersec_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/refined_intersec", 10, true);
      cluster_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/intersec_clusters", 10, true);
      all_hulls_vis_pub_ = nhandle_.advertise<visualization_msgs::MarkerArray> ("transObjRec/intersec_cluster_hulls", 10, true);
      result_pub_ = nhandle_.advertise<object_recognition_msgs::RecognizedObjectArray> ("transObjRec/trans_recon_results", 10, false);

      // subscribe to voxelized transparent object information
      voxelized_info_sub_ = nhandle_.subscribe ("transObjRec/voxelized_info", 5, &ExTraReconstructedObject::cluster_refine_cb, this);

      ROS_INFO ("created ExTraReconstructedObject and subscribed to topic");

      setUpVisMarker ();

      db_type = "{\"type\":\"empty\"}";
    };

    void cluster_refine_cb (const transparent_object_reconstruction::VoxelizedTransObjInfo &trans_obj_info)
    {
      // convert from sensor_msgs::PointCloud2 to templated pcl pointcloud
      LabelCloudPtr voxelized_intersec (new LabelCloud);
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL (trans_obj_info.voxel_centers, pcl_pc2);
      pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
      pcl::fromPCLPointCloud2 (pcl_pc2, *voxelized_intersec);
      pcl::console::setVerbosityLevel (pcl::console::L_INFO);
      std_msgs::Header header;
      header = trans_obj_info.voxel_centers.header;
      pcl_conversions::toPCL (header, voxelized_intersec->header);

      ROS_INFO ("received VoxelizedTransObjInfo, containing cloud with %lu points",
          voxelized_intersec->points.size ());

      static int call_counter = 0;

      // check for bag loop
      static bag_loop_check::BagLoopCheck bagloop;
      if (bagloop)
      {
        param_handle_.param<int> ("angle_resolution", angle_resolution_, ANGLE_RESOLUTION);
        param_handle_.param<int> ("opening_angle", opening_angle_, OPENING_ANGLE);
        param_handle_.param<float> ("median_fraction", median_fraction_, MEDIAN_FRACTION);
        param_handle_.param<bool> ("write_visualization", write_visualization_, false);
      }

      ROS_DEBUG ("ExTra-callback params: angle_resolution_ %i, opening_angle_ %i, median_fraction_ %f, write_visualization_ %s", angle_resolution_, opening_angle_, median_fraction_, write_visualization_ ? "true" : "false");

      // clear old marker
      clear_marker_array_.markers.front ().header = trans_obj_info.voxel_centers.header;
      all_hulls_vis_pub_.publish (clear_marker_array_);

      // perform clustering on the voxelized transparent object information
      pcl::search::KdTree<LabelPoint>::Ptr tree (new pcl::search::KdTree<LabelPoint>);
      tree->setInputCloud (voxelized_intersec);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<LabelPoint> ec;
      ec.setClusterTolerance (cluster_tolerance_);
      ec.setMinClusterSize (min_cluster_size_);
      ec.setMaxClusterSize (max_cluster_size_);
      ec.setSearchMethod (tree);
      ec.setInputCloud (voxelized_intersec);
      ec.extract (cluster_indices);

      size_t total_cluster_points = 0;
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
        extract_object_indices.setInputCloud (voxelized_intersec);
        extract_object_indices.setIndices (pcl::PointIndices::Ptr (new pcl::PointIndices(*it)));
        LabelCloudPtr tmp (new LabelCloud);
        extract_object_indices.filter (*tmp);
        output.push_back (tmp);
        total_cluster_points += tmp->points.size ();
      }
  
      LabelCloudPtr all_refined_clusters (new LabelCloud);
      all_refined_clusters->points.reserve (total_cluster_points);

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
      transparent_recon_objs->header = trans_obj_info.voxel_centers.header;
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
        output[i]->header = voxelized_intersec->header; // explicitly copy header information
        h += color_increment;
        total_points += output[i]->points.size ();

        // ====== refinement filter of the extracted clusters =====
        ROS_INFO ("started working on a cluster with %lu points", output[i]->points.size ());


        // ===== bin visualization =====
        std::stringstream img_ss;
        std::ofstream img;
        if (write_visualization_)
        {
          img_ss << "bit_arrays_frame" << std::setw (3) << std::setfill ('0') << call_counter
            << "_cluster" << std::setw (3) << std::setfill ('0') << i << ".pbm";
          img.open (img_ss.str ().c_str ());
          img << "P1" << "\n#Visualization of viewpoint intervals of cluster " << i << " in frame "
          << call_counter << "\n";
        }
        // ===== bin visualization =====

        // collect all labels in the current cluster and the intervals for each voxel / leaf
        // storage for the approximate cluster center
        Eigen::Vector3d approx_cluster_center = Eigen::Vector3d::Zero ();
        std::multimap<size_t, std::pair<size_t, boost::icl::interval_set<int> > > interval_map;
        LabelCloudPtr refined_voxel_centers (new LabelCloud);
        std::set<uint32_t> all_labels_in_cluster;
        std::vector<int>::const_iterator cluster_index_it = cluster_indices[i].indices.begin ();
        while (cluster_index_it != cluster_indices[i].indices.end ())
        {
          // retrieve labels for the current voxel
          std::vector<uint32_t>::const_iterator label_it = trans_obj_info.voxel_labels[*cluster_index_it].labels.begin ();
          // add voxel labels to the set of cluster labels
          while (label_it != trans_obj_info.voxel_labels[*cluster_index_it].labels.end ())
          {
            all_labels_in_cluster.insert (*label_it++);
          }

          // retrieve the viewpoint intervals for the current voxel
          boost::icl::interval_set<int> voxel_vp_interval;
          convertVoxelViewpointIntervals2ICLIntervalSet (trans_obj_info.voxel_intervals[*cluster_index_it],
              voxel_vp_interval);
          interval_map.insert (std::pair<size_t, std::pair<size_t, boost::icl::interval_set<int> > >
              (voxel_vp_interval.size (), std::pair<size_t, boost::icl::interval_set<int> > (*cluster_index_it,voxel_vp_interval)));

          // add voxel center to approximate cluster center
          approx_cluster_center += convert<Eigen::Vector3d, LabelPoint> (voxelized_intersec->points[*cluster_index_it]);

          cluster_index_it++;
        }
        // compute the approximate cluster center
        approx_cluster_center /= cluster_indices[i].indices.size ();

        std::multimap<size_t, std::pair<size_t, boost::icl::interval_set<int> > >::const_iterator map_it
          = interval_map.begin ();
        // ===== bin visualization =====
        if (write_visualization_)
        {
          img << "# cluster contained the following labels: ";
          // add information about available labels to image file
          std::set<uint32_t>::const_iterator cluster_label_it = all_labels_in_cluster.begin ();
          while (cluster_label_it != all_labels_in_cluster.end ())
          {
            img << *cluster_label_it++ << " ";
          }
          img << std::endl;
          img << "# approximated cluster center: " << approx_cluster_center[0] << ", "
            << approx_cluster_center[1] << ", " << approx_cluster_center[2] << std::endl;
          img << angle_resolution_ << " " << output[i]->points.size () << std::endl;

          while (map_it != interval_map.end ())
          {
            // generate string from interval set
            std::stringstream img_line_ss;
            std::vector<int> zero_line (angle_resolution_, 0);
            boost::icl::interval_set<int>::const_iterator i_set_iterator = map_it->second.second.begin ();
            while (i_set_iterator != map_it->second.second.end ())
            {
              for (int k = i_set_iterator->lower (); k <= i_set_iterator->upper (); ++k)
              {
                zero_line[k] = 1;
              }
              i_set_iterator++;
            }
            for (size_t k = 0; k < zero_line.size (); ++k)
            {
              img_line_ss << zero_line[k] << " ";
            }
            img_line_ss << std::endl;
            // add image line to image
            img << img_line_ss.str ();
            map_it++;
          }

          img.flush ();
          img.close ();
          ROS_INFO ("wrote visualization file '%s'.", img_ss.str ().c_str ());
          map_it = interval_map.begin ();
        }
        // ===== bin visualization =====

        // get median viewpoint coverage of the current cluster
        for (size_t j = 0; j < interval_map.size () / 2; ++j)
        {
          map_it++;
        }
        size_t median = map_it->first;
        size_t median_vp_threshold = static_cast<size_t> (median_fraction_ * median);
        // iterate over all leaves that are to be ignored (because they cover less than the desired threshold)
        map_it = interval_map.begin ();
        while (map_it->first < median_vp_threshold)
        {
          map_it++;
        }
        // store all other voxel centers / leaves as refined voxel centers
        while (map_it != interval_map.end ())
        {
          refined_voxel_centers->points.push_back (voxelized_intersec->points[map_it->second.first]);
          map_it++;
        }
        ROS_INFO ("determined that %lu of %lu initial points belong to transparent object",
            refined_voxel_centers->points.size (), output[i]->points.size ());

        // TODO: try mean based post-filtering instead of median

        // set header and adapt the dimensions of the refined clouds
        refined_voxel_centers->header = output[i]->header;
        refined_voxel_centers->width = refined_voxel_centers->points.size ();
        refined_voxel_centers->height = 1;

        // color and label points of the current cluster
        LabelCloud::VectorType::iterator vc_it = refined_voxel_centers->points.begin ();
        while (vc_it != refined_voxel_centers->points.end ())
        {
          vc_it->r = static_cast<uint8_t> (r * std::numeric_limits<uint8_t>::max ());
          vc_it->g = static_cast<uint8_t> (g * std::numeric_limits<uint8_t>::max ());
          vc_it->b = static_cast<uint8_t> (b * std::numeric_limits<uint8_t>::max ());
          vc_it->label = static_cast<uint32_t> (i);
          vc_it++;
        }

        // all colored refined points of current cluster to all cluster cloud
        all_refined_clusters->points.insert (all_refined_clusters->points.end (),
            refined_voxel_centers->points.begin (), refined_voxel_centers->points.end ());

        // ====== refinement filter of the extracted clusters =====

        // compute convex hull for current cluster refinement
        LabelCloudPtr convex_hull (new LabelCloud);
        std::vector<pcl::Vertices> polygons;
        pcl::ConvexHull<LabelPoint> c_hull;
        c_hull.setInputCloud (refined_voxel_centers);
        c_hull.setDimension (3);
        c_hull.reconstruct (*convex_hull, polygons);

        // create marker for the convex hull
        std::stringstream ss;
        ss << "convex_hull_intersec_cluster" << std::setw (3) << std::setfill ('0') << i;
        visualization_msgs::Marker curr_hull_marker (hull_marker_);
        curr_hull_marker.id = i;
        curr_hull_marker.ns = ss.str ();
        curr_hull_marker.color.r = r;
        curr_hull_marker.color.g = g;
        curr_hull_marker.color.b = b;

        Eigen::Vector3f cog;
        shape_msgs::Mesh curr_mesh;
        tesselate3DConvexHull<LabelPoint> (convex_hull, polygons, curr_hull_marker, cog, curr_mesh);
        curr_hull_marker.header = trans_obj_info.voxel_centers.header;
        // TODO: time stamp of header should be updated, right?
        curr_hull_marker.header.stamp = ros::Time::now ();

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
        pcl::toROSMsg (*output[i], pc2);
        o.point_clouds.push_back (pc2);
        // add the mesh to recognized obj
        o.bounding_mesh = curr_mesh;

        // TODO: should covariance be set to identity matrix or remain 0-matrix?

        transparent_recon_objs->objects.push_back (o);
      }
      
      // publish the convex hulls of all clusters
      all_hulls_vis_pub_.publish (all_hulls);

      result_pub_.publish (transparent_recon_objs);

      // publish visualization of the refined cluster centers
      std_msgs::Header all_refined_header = trans_obj_info.voxel_centers.header;
      all_refined_header.stamp = ros::Time::now ();
      pcl_conversions::toPCL (all_refined_header, all_refined_clusters->header);
      all_refined_clusters->width = all_refined_clusters->points.size ();
      all_refined_clusters->height = 1;

      // publish all clusters together in 1 point cloud, colored and labeled differently
      refined_intersec_pub_.publish (all_refined_clusters);

      call_counter++;

      ROS_INFO ("finished ExTraction callback; published %lu transparent clusters", output.size ());
    };

  protected:
    ros::NodeHandle nhandle_;
    ros::NodeHandle param_handle_;
    ros::Subscriber voxelized_info_sub_;

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
    bool write_visualization_;

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
