#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/ModelCoefficients.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>

#include <iostream>
#include <iomanip>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/Holes.h>
#include <transparent_object_reconstruction/tools.h>
#include <transparent_object_reconstruction/HoleIntersectorReset.h>

typedef pcl::octree::OctreePointCloud<LabelPoint> LabelOctree;
typedef pcl::octree::OctreeContainerPointIndices LeafContainer;


class HoleIntersector
{
  public:
    HoleIntersector (float octree_r, size_t min_points, std::string tabletop_frame) :
      octree_resolution_ (octree_r),
      min_leaf_points_ (min_points),
      tabletop_frame_ (tabletop_frame)
    {
      setUpVisMarkers ();

      vis_pub_ = nhandle_.advertise<visualization_msgs::Marker>( "intersec_visualization", 10, true);
      all_frusta_pub_ = nhandle_.advertise<visualization_msgs::MarkerArray>( "frusta_visualization", 10, true);

      intersec_pub_ = nhandle_.advertise<LabelCloud> ("transparent_object_intersection", 10, true);

      reset_service_ = nhandle_.advertiseService ("collector_reset", &HoleIntersector::reset, this); 

      hole_sub_ = nhandle_.subscribe ("/table_holes", 1, &HoleIntersector::add_holes_cb, this);

      octree_.reset (new LabelOctree (octree_resolution_));
      all_frusta_ = boost::make_shared<LabelCloud> ();
      intersec_cloud_ = boost::make_shared<LabelCloud> ();

    };

    void add_holes_cb (const transparent_object_reconstruction::Holes::ConstPtr &holes)
    {
      ROS_DEBUG ("went into callback, msg size: %lu", holes->convex_hulls.size ());

      if (!holes->convex_hulls.size () > 0)
      {
        ROS_WARN ("received empty Holes message; ignoring");
        return;
      }

      // check for backloop
      //TODO: refine this hack later
      if (collected_views_.size () > 0 &&
          (holes->convex_hulls.front ().header.stamp - collected_views_.back ().stamp) < ros::Duration (-1.0))
      {
        ROS_INFO ("detected backloop");
        transparent_object_reconstruction::HoleIntersectorReset::Request req;
        transparent_object_reconstruction::HoleIntersectorReset::Response res;
        this->reset (req, res);
      }

      // since curious table explorer publishes every table view exactly once, we can omit check
      /*
      // compare the timestamps of the headers - if the timestamps are less than 1 second apart, we assume same frame
      std::vector<std_msgs::Header>::const_iterator view_it = collected_views_.begin ();
      while (view_it != collected_views_.end ())
      {
        ros::Duration d = holes->convex_hulls.front ().header.stamp - view_it->stamp;
        ros::Duration pos_sec (1.0);
        ros::Duration neg_sec (-1.0);
        if (d >= neg_sec && d <= pos_sec)
        {
          ROS_WARN ("Holes msg for frame '%s' already collected. Ignoring new message.",
              holes->convex_hulls.front ().header.frame_id.c_str ());
          return;
        }
        view_it++;
      }
      */
      // since the view was not present so far, add it to the collection
      collected_views_.push_back (holes->convex_hulls.front ().header);
      
      ROS_DEBUG ("added header to collected_views_");
      ROS_DEBUG ("frame_id: %s, tabletop_frame: %s",
          holes->convex_hulls.front ().header.frame_id.c_str (),
          tabletop_frame_.c_str ());

      if (!tflistener_.waitForTransform (holes->convex_hulls.front ().header.frame_id,
          tabletop_frame_,
          holes->convex_hulls.front ().header.stamp, ros::Duration (600.0)))
      {
        ROS_ERROR ("Didn't retrieve a transfrom between %s and %s",
            holes->convex_hulls.front ().header.frame_id.c_str (),
            tabletop_frame_.c_str ());
        return;
      }
      tf::StampedTransform tf_transform;

      ROS_DEBUG ("finished wait for transform");

      try
      {
        // retrieve transformation
        tflistener_.lookupTransform (tabletop_frame_,
            holes->convex_hulls[0].header.frame_id,
            holes->convex_hulls[0].header.stamp,
            tf_transform);
      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN ("Transform unavailable: %s", ex.what ());
        return;
      }

      ROS_DEBUG ("got transform");

      // retrieve the label for the new points
      uint32_t current_label = transformed_holes_.size ();
      // convert the tf transform to Eigen...
      Eigen::Affine3d eigen_transform;
      tf::transformTFToEigen (tf_transform, eigen_transform);

      // Transform the origin of the frame where the holes were recorded
      Eigen::Vector3d transformed_origin;
      pcl::transformPoint (Eigen::Vector3d::Zero (), transformed_origin, eigen_transform);

      std::vector<LabelCloudPtr> current_holes;
      current_holes.reserve (holes->convex_hulls.size ());

      // transform all convex hulls point clouds into the table frame (aligned with x-y-plane)
      for (size_t i = 0; i < holes->convex_hulls.size (); ++i)
      {
        // convert sensor_msgs::PointCloud2 to PCLPointCloud2 to pcl::PointCloud<T>
        LabelCloudPtr hole_hull (new LabelCloud);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL (holes->convex_hulls[i], pcl_pc2);
        pcl::fromPCLPointCloud2 (pcl_pc2, *hole_hull);

        ROS_DEBUG ("finished conversion, hole hull size: %lu",
            hole_hull->points.size ());

        // ----- sample inside of hole -----

        // transform into table frame (tabletop aligned with x-y-plane)
        LabelCloudPtr xy_hole_hull (new LabelCloud);
        pcl::transformPointCloud (*hole_hull, *xy_hole_hull, eigen_transform);

        ROS_DEBUG ("successfully transformed pointcloud, Yay!");

        geometry_msgs::Point tmp_point;
        tmp_point.x = transformed_origin[0];
        tmp_point.y = transformed_origin[1];
        tmp_point.z = transformed_origin[2];
        if (tesselateConeOfHull<LabelPoint> (xy_hole_hull, basic_frustum_marker_, &tmp_point))
        {
          basic_frustum_marker_.id = i;
          frusta_marker_.markers.push_back (basic_frustum_marker_);
        }

        // create grid for sampling
        pcl::VoxelGrid<LabelPoint> grid;
        grid.setSaveLeafLayout (true);
        grid.setInputCloud (xy_hole_hull);
        grid.setLeafSize (0.005f, 0.005f, 1.0f); // TODO: make adaptable
        grid.setDownsampleAllData (false);
        grid.setMinimumPointsNumberPerVoxel (1);
        grid.setFilterLimitsNegative (true);

        ROS_DEBUG ("VoxelGrid created");

        LabelCloudPtr inliers (new LabelCloud);
        grid.filter (*inliers);

        // retrieve the hull in terms of 2D grid coordinates
        std::vector<Eigen::Vector2i> hull_polygon;
        hull_polygon.reserve (xy_hole_hull->points.size ());
        LabelCloud::VectorType::const_iterator h_it = xy_hole_hull->points.begin ();
        Eigen::Vector3i grid_coords, bbox_min, bbox_max;
        Eigen::Vector2i query_point;
        while (h_it != xy_hole_hull->points.end ())
        {
          grid_coords = grid.getGridCoordinates (h_it->x, h_it->y, h_it->z);
          hull_polygon.push_back (Eigen::Vector2i (grid_coords[0], grid_coords[1]));
          h_it++;
        }

        ROS_DEBUG ("retrieved grid coords, nr_grid cells: %lu", hull_polygon.size ());

        // get lower and upper corners of VoxelGrid
        bbox_min = grid.getMinBoxCoordinates ();
        bbox_max = grid.getMaxBoxCoordinates ();

        // get all grid coordinates that are inside the 2D polygon to create the sampled hole
        Eigen::Vector3f leaf_size = grid.getLeafSize ();
        LabelPoint inside_point;
        inside_point.z = 0.0f;
        inside_point.label = current_label;
        LabelCloudPtr xy_hole_sample_cloud (new LabelCloud);
        xy_hole_sample_cloud->points.reserve ((bbox_max[0] - bbox_min[0]) * (bbox_max[1] - bbox_min[1]));
        for (int u = bbox_min[0]; u < bbox_max[0]; ++u)
        {
          query_point[0] = u;
          inside_point.x = (u + .5f) * leaf_size[0];
          for (int v = bbox_min[1]; v < bbox_max[1]; ++v)
          {
            query_point[1] = v;
            if (pointInPolygon2D (hull_polygon, query_point))
            {
              inside_point.y = (v + .5f) * leaf_size[1];
              xy_hole_sample_cloud->points.push_back (inside_point);
            }
          }
        }

        ROS_DEBUG ("created hole sample in x_y_plane, size: %lu, dims: %ix%i",
            xy_hole_sample_cloud->points.size (), bbox_max[0] - bbox_min[0], bbox_max[1] - bbox_min[1]);
        
        // store the convex hull in the tabletop frame (with point labels)
        current_holes.push_back (xy_hole_sample_cloud);
        // store the transform
        transforms_.push_back (eigen_transform);

        // transform sampled hole back into its original frame (where sensor is placed at (0,0,0))
        LabelCloudPtr hole_sample_cloud (new LabelCloud);
        pcl::transformPointCloud (*xy_hole_sample_cloud, *hole_sample_cloud, eigen_transform.inverse ());

        // ----- create frustum -----
        LabelCloudPtr frustum (new LabelCloud);
        createSampleRays (hole_sample_cloud, frustum, leaf_size[0]); //TODO: sampling with hole_sample_cloud, not with xy_hole_sample_cloud!

        ROS_DEBUG ("created frustum, size: %lu", frustum->points.size ());
        frustum->header = hole_sample_cloud->header;

        if (frustum->points.size () > 0)
        {
          // add currently used label to the set of available labels
          available_labels_.insert (current_label);
          // transform frustum into global frame
          LabelCloudPtr transformed_frustum (new LabelCloud);
          pcl::transformPointCloud (*frustum, *transformed_frustum, eigen_transform);

          // add frustum to collection of all frusta
          all_frusta_->points.reserve (all_frusta_->points.size () +
              transformed_frustum->points.size ());
          all_frusta_->points.insert (all_frusta_->points.end (),
              transformed_frustum->points.begin (), transformed_frustum->points.end ());
          // adapt dimensions of point cloud
          all_frusta_->width = all_frusta_->points.size ();
          all_frusta_->height = 1;
          ROS_DEBUG ("inserted frustum points into all_frusta_");
        }
      }
      // store all convex hulls of the current Holes msgs (aligned to tabletop)
      transformed_holes_.push_back (current_holes);
      
      // compute intersection
      this->computeIntersection ();

      ROS_DEBUG ("Finished callback, intersections and visualization for %lu views are computed", collected_views_.size ());
    };

    void computeIntersection (void)
    {
      intersec_cloud_->points.clear ();
      if (available_labels_.size () < 1)
      {
        ROS_WARN ("called 'computeIntersection ()', but no label exists; Exiting intersection computation");
        return;
      }
      // clear old content from octree and intersection cloud
      octree_->deleteTree ();
      octree_->setInputCloud (all_frusta_);
      octree_->addPointsFromInputCloud ();

      // iterate over all leafs and check which are filled with points of all labels
      LabelOctree::LeafNodeIterator leaf_it;
      leaf_it = octree_->leaf_begin ();
      pcl::PointIndices::Ptr leaf_point_indices (new pcl::PointIndices);
      LabelCloudPtr leaf_cloud (new LabelCloud);
      pcl::ExtractIndices<LabelPoint> extract;
      extract.setInputCloud (all_frusta_);

      // get nr occupied voxels
      // TODO: check if there is a more efficient way to do this
      LabelCloud::VectorType leaf_centers;
      LabelCloud::VectorType::const_iterator p_it;
      octree_->getOccupiedVoxelCenters (leaf_centers);

      // clear old contents from message markers
      intersec_marker_.points.clear ();
      non_intersec_marker_.points.clear ();
      intersec_marker_.points.reserve (leaf_centers.size ());
      non_intersec_marker_.points.reserve (leaf_centers.size ());

      // iterate over all leaves to check which belongs to the intersection
      intersec_cloud_->points.reserve (all_frusta_->points.size ());
      size_t nr_leaves, filled_leaves, intersec_leaves;
      nr_leaves = filled_leaves = intersec_leaves = 0;
      Eigen::Vector3f min, max, center;
      geometry_msgs::Point voxel_center;
      while (leaf_it != octree_->leaf_end ())
      {
        nr_leaves++;
        // remove old point indices
        leaf_point_indices->indices.clear ();

        // retreave container for the current leaf
        LeafContainer &container = leaf_it.getLeafContainer ();
        container.getPointIndices (leaf_point_indices->indices);

        // check if enough points in leaf
        if (leaf_point_indices->indices.size () > min_leaf_points_)
        {
          filled_leaves++;
          extract.setIndices (leaf_point_indices);
          extract.filter (*leaf_cloud);

          // retrieve the center point of the current leaf
          octree_->getVoxelBounds (leaf_it, min, max);
          center = min + max;
          center /= 2.0f;
          voxel_center.x = center[0];
          voxel_center.y = center[1];
          voxel_center.z = center[2];

          if (isLeafInIntersection (*leaf_cloud, available_labels_))
          {
            intersec_marker_.points.push_back (voxel_center);

            intersec_leaves++;
            p_it = leaf_cloud->points.begin ();
            while (p_it != leaf_cloud->points.end ())
            {
              intersec_cloud_->points.push_back (*p_it++);
            }
          }
          else
          {
            non_intersec_marker_.points.push_back (voxel_center);
          }
        }
        leaf_it++;
      }
      if (intersec_cloud_->points.size () > 0)
      {
        this->publish_intersec ();
      }

      this->publish_markers ();
    };

    void publish_intersec (void)
    {
      if (intersec_cloud_ != NULL)
      {
        // create Header with appropriate frame and time stamp
        std_msgs::Header header;
        header.frame_id = tabletop_frame_;
        header.stamp = ros::Time::now ();
        pcl_conversions::toPCL (header, intersec_cloud_->header);

        // set width and height of cloud
        intersec_cloud_->height = 1;
        intersec_cloud_->width = intersec_cloud_->points.size ();

        // publish
        intersec_pub_.publish (intersec_cloud_);

        // TODO: publish also as mesh or something similar
      }
    };

    void publish_markers (void)
    {
      intersec_marker_.header.stamp = ros::Time::now ();
      intersec_marker_.header.frame_id = tabletop_frame_;
      non_intersec_marker_.header.stamp = ros::Time::now ();
      non_intersec_marker_.header.frame_id = tabletop_frame_;
      // publish visualization marker
      vis_pub_.publish (intersec_marker_);
      vis_pub_.publish (non_intersec_marker_);

      // recolor and publish Marker array for occlusion frusta
      frame_change_indices.push_back (frusta_marker_.markers.size ());
      float h, r, g, b, color_increment;
      color_increment = 360.0f / static_cast<float> (frame_change_indices.size ());
      h = 0.0f;
      size_t start_index, end_index;
      start_index = 0;
      std::stringstream ss;
      ROS_DEBUG ("frusta_marker_.markers.size (): %lu", frusta_marker_.markers.size ());
      for (size_t i = 0; i < frame_change_indices.size (); ++ i)
      {
        end_index = frame_change_indices[i];
        hsv2rgb (h, r, g, b);
        ss << "frame_" << i;
        ROS_DEBUG ("start_index: %lu, end_index: %lu", start_index, end_index);
        for (size_t j = start_index; j < end_index; ++j)
        {
          frusta_marker_.markers[j].ns = ss.str ();
          frusta_marker_.markers[j].color.r = r;
          frusta_marker_.markers[j].color.g = g;
          frusta_marker_.markers[j].color.b = b;
          frusta_marker_.markers[j].header.stamp = ros::Time::now ();
        }
        ss.str ("");
        h += color_increment;
        start_index = end_index;
      }
      all_frusta_pub_.publish (frusta_marker_);

      ROS_INFO ("published markers, currently %lu views collected", collected_views_.size ());
    };

    bool reset (transparent_object_reconstruction::HoleIntersectorReset::Request &req,
        transparent_object_reconstruction::HoleIntersectorReset::Response &res)
    {
      // reset collected holes an the transformations to tabletop frame...
      transformed_holes_.clear ();
      transforms_.clear ();
      // ...reset the combined cloud holding all frusta and the intersection...
      all_frusta_->points.clear ();
      all_frusta_->width = all_frusta_->height = 0;
      intersec_cloud_->points.clear ();
      intersec_cloud_->width = intersec_cloud_->height = 0;
      // ...reset the labes used up until now...
      available_labels_.clear ();
      // ...reset the collected views...
      collected_views_.clear ();
      // ...reset the markers...
      intersec_marker_.points.clear ();
      non_intersec_marker_.points.clear ();
      // ...reset and clear marker array...
      clear_marker_array_.markers.front ().header.stamp = ros::Time::now ();
      all_frusta_pub_.publish (clear_marker_array_);
      frusta_marker_.markers.clear ();
      frame_change_indices.clear ();
      // ...and exit
      ROS_INFO ("Reset HoleIntersector");
      return true;
    };


  private:
    size_t min_leaf_points_;
    float octree_resolution_;
    std::string tabletop_frame_;

    void setUpVisMarkers (void)
    {
      // set up visualization marker
      intersec_marker_.ns = "intersec";
      intersec_marker_.id = 0;
      intersec_marker_.type = visualization_msgs::Marker::CUBE_LIST;
      intersec_marker_.action = visualization_msgs::Marker::ADD;
      intersec_marker_.pose.position.x = 0;
      intersec_marker_.pose.position.y = 0;
      intersec_marker_.pose.position.z = 0;
      intersec_marker_.pose.orientation.x = 0.0;
      intersec_marker_.pose.orientation.y = 0.0;
      intersec_marker_.pose.orientation.z = 0.0;
      intersec_marker_.pose.orientation.w = 1.0;
      intersec_marker_.scale.x = octree_resolution_;
      intersec_marker_.scale.y = octree_resolution_;
      intersec_marker_.scale.z = octree_resolution_;
      intersec_marker_.color.a = 0.25;
      intersec_marker_.color.r = 0.0;
      intersec_marker_.color.g = 1.0;
      intersec_marker_.color.b = 0.0;

      non_intersec_marker_ = visualization_msgs::Marker (intersec_marker_);
      non_intersec_marker_.ns = "non_intersec";
      non_intersec_marker_.id = 1;
      non_intersec_marker_.color.r = 1.0;
      non_intersec_marker_.color.g = 0.0;

      // set up basic frustum marker
      basic_frustum_marker_ = visualization_msgs::Marker (intersec_marker_);
      basic_frustum_marker_.header.frame_id = tabletop_frame_;
      basic_frustum_marker_.ns = "frusta_cone";
      basic_frustum_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
      basic_frustum_marker_.action = visualization_msgs::Marker::ADD;
      basic_frustum_marker_.color.a = 0.4;
      basic_frustum_marker_.scale.x = 1.0;
      basic_frustum_marker_.scale.y = 1.0;
      basic_frustum_marker_.scale.z = 1.0;

      visualization_msgs::Marker clear_marker (intersec_marker_);
      clear_marker.header.frame_id = "map";
      // DELETEALL is not officially around before jade, addressed it by value
      clear_marker.action = 3;
      clear_marker_array_.markers.push_back (clear_marker);


    };

  protected:
    ros::NodeHandle nhandle_;
    ros::Subscriber hole_sub_;
    
    ros::Publisher vis_pub_;
    ros::Publisher intersec_pub_;
    ros::Publisher all_frusta_pub_;

    ros::ServiceServer reset_service_;

    tf::TransformListener tflistener_;
    std::vector<std::vector<LabelCloudPtr> > transformed_holes_;
    std::vector<Eigen::Affine3d> transforms_;
    LabelCloudPtr all_frusta_;
    LabelCloudPtr intersec_cloud_;
    std::set<uint32_t> available_labels_;
    std::vector<std_msgs::Header> collected_views_;
    std::vector<size_t> frame_change_indices;

    visualization_msgs::Marker intersec_marker_;
    visualization_msgs::Marker non_intersec_marker_;
    visualization_msgs::Marker basic_frustum_marker_;
    visualization_msgs::MarkerArray frusta_marker_;
    visualization_msgs::MarkerArray clear_marker_array_;

    LabelOctree::Ptr octree_;

    static void addLabelToCloud (LabelCloudPtr &cloud, uint32_t label)
    {
      LabelCloud::VectorType::iterator p_it = cloud->points.begin ();
      while (p_it != cloud->points.end ())
      {
        p_it->label = label;
        p_it++;
      }
    };
    
    static bool isLeafInIntersection (const LabelCloud &leaf_cloud,
        const std::set<uint32_t> &labels)
    {
      std::map<uint32_t, size_t> leaf_labels;
      std::map<uint32_t, size_t>::iterator detected_label;
      std::set<uint32_t>::const_iterator label_it = labels.begin ();

      while (label_it != labels.end ())
      {
        leaf_labels.insert (std::pair<uint32_t, size_t> (*label_it, 0));
        label_it++;
      }

      LabelCloud::VectorType::const_iterator p_it;
      p_it = leaf_cloud.points.begin ();
      while (p_it != leaf_cloud.points.end ())
      {
        detected_label = leaf_labels.find (p_it->label);
        if (detected_label != leaf_labels.end ())
        {
          detected_label->second++; // increase counter for detected label
        }
        else
        {
          ROS_WARN ("isLeafInIntersection (): found label that was not in set of available labels");
        }
        p_it++;
      }

      bool missing_label = false;
      size_t min_points_per_label = 1; // TODO: change this value dynammically dependent on total number of points in leaf?

      detected_label = leaf_labels.begin ();
      while (detected_label != leaf_labels.end ())
      {
        if (detected_label->second < min_points_per_label)
        {
          missing_label = true;
          return false;
        }
        detected_label++;
      }
      if (!missing_label)
        return true;
      return false;
    };

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "HoleIntersector");

  HoleIntersector h (0.005f, 1, "/tracked_table");

  ros::spin ();

  return 0;
}
