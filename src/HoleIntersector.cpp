#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
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
#include <algorithm>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/Holes.h>
#include <transparent_object_reconstruction/tools.h>
#include <transparent_object_reconstruction/HoleIntersectorReset.h>

#include <bag_loop_check/bag_loop_check.hpp>

typedef pcl::octree::OctreePointCloud<LabelPoint> LabelOctree;
typedef pcl::octree::OctreeContainerPointIndices LeafContainer;

class HoleIntersector
{
  public:
    HoleIntersector (float octree_r, size_t min_points, std::string tabletop_frame, std::string map_frame) :
      octree_resolution_ (octree_r),
      min_leaf_points_ (min_points),
      tabletop_frame_ (tabletop_frame),
      map_frame_ (map_frame)
    {
      setUpVisMarkers ();

      param_handle_ = ros::NodeHandle ("~");

      // retrieve minimal ratio of detected labels or use default parameter
      param_handle_.param<int> ("angle_resolution", angle_resolution_, ANGLE_RESOLUTION);
      param_handle_.param<int> ("opening_angle", opening_angle_, OPENING_ANGLE);
      param_handle_.param<int> ("min_bin_marks", min_bin_marks_, MIN_BIN_MARKS);

      vis_pub_ = nhandle_.advertise<visualization_msgs::MarkerArray>( "transObjRec/intersec_visualization", 10, true);
      all_frusta_pub_ = nhandle_.advertise<visualization_msgs::MarkerArray>( "transObjRec/frusta_visualization", 10, true);

      intersec_pub_ = nhandle_.advertise<LabelCloud> ("transObjRec/intersection", 10, true);

      reset_service_ = nhandle_.advertiseService ("transObjRec/HoleIntersector_reset", &HoleIntersector::reset, this);

      hole_sub_ = nhandle_.subscribe ("table_holes", 1, &HoleIntersector::add_holes_cb, this);

      octree_.reset (new LabelOctree (octree_resolution_));
      all_frusta_ = boost::make_shared<LabelCloud> ();
      intersec_cloud_ = boost::make_shared<LabelCloud> ();

      // indicate that reference bounding box for octree isn't set yet
      reference_bb_set_ = false;

    };

    void add_holes_cb (const transparent_object_reconstruction::Holes::ConstPtr &holes)
    {
      ROS_DEBUG ("went into callback, msg size: %lu", holes->convex_hulls.size ());

      if (!holes->convex_hulls.size () > 0)
      {
        ROS_WARN ("received empty Holes message; ignoring");
        return;
      }

      ros::Time cb_start_time = ros::Time::now ();

      // check for bag loop
      static bag_loop_check::BagLoopCheck bagloop;
      if (bagloop && collected_views_.size () > 0)
      {
        ROS_INFO ("Detected bag loop; Reseting HoleIntersector");
        transparent_object_reconstruction::HoleIntersectorReset::Request req;
        transparent_object_reconstruction::HoleIntersectorReset::Response res;
        this->reset (req, res);
        // check if a different parameters were provided
        param_handle_.param<int> ("angle_resolution", angle_resolution_, ANGLE_RESOLUTION);
        param_handle_.param<int> ("opening_angle", opening_angle_, OPENING_ANGLE);
        param_handle_.param<int> ("min_bin_marks", min_bin_marks_, MIN_BIN_MARKS);
      }

      ROS_DEBUG ("INTERSECTOR-callback params: angle_resolution_ %i, opening_angle_ %i, min_bin_marks_ %i",
          angle_resolution_, opening_angle_, min_bin_marks_);

      // since the view was not present so far, add it to the collection
      collected_views_.push_back (holes->convex_hulls.front ().header);
      
      ROS_DEBUG ("added header to collected_views_");
      ROS_DEBUG ("frame_id: %s, tabletop_frame: %s",
          holes->convex_hulls.front ().header.frame_id.c_str (),
          tabletop_frame_.c_str ());

      if (!tflistener_.waitForTransform (holes->convex_hulls.front ().header.frame_id,
          tabletop_frame_,
          holes->convex_hulls.front ().header.stamp, ros::Duration (10.0)))
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
            current_yaw_ = tf::getYaw (tf_transform.getRotation ());
      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN ("Transform unavailable: %s", ex.what ());
        return;
      }
      ROS_DEBUG ("got transform");

      if (map_frame_.compare (tabletop_frame_) != 0)
      {
        if (!tflistener_.waitForTransform (tabletop_frame_, map_frame_,
              holes->convex_hulls.front ().header.stamp, ros::Duration (50.0)))
        {
          ROS_ERROR ("Didn't retrieve a transfrom between '%s' and %s",
              tabletop_frame_.c_str (),
              map_frame_.c_str ());
          return;
        }
        ROS_DEBUG ("finished waiting for transform between %s and %s",
            tabletop_frame_.c_str (),
            map_frame_.c_str ());

        try
        {
          tflistener_.lookupTransform (map_frame_,
              tabletop_frame_,
              holes->convex_hulls[0].header.stamp,
              table_to_map_);
          tf::transformTFToEigen (table_to_map_, table_to_map_transform_);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN ("Transform between %s and %s unavailable: %s",
              tabletop_frame_.c_str (), map_frame_.c_str (), ex.what ());
          return;
        }
      }

      // retrieve the label for the new points
      // compute the current label from the used orientation
      double yaw_in_degrees = pcl::rad2deg (current_yaw_);
      while (yaw_in_degrees < 0.0f)
      {
        yaw_in_degrees += 360.0f;
      }
      // scale to desired resolution
      uint32_t current_label = static_cast<uint32_t> (yaw_in_degrees * 360.0f / angle_resolution_);
      all_labels_.insert (current_label);

      std::set<uint32_t>::const_iterator label_it = all_labels_.begin ();
      std::cout  << "all_labels: ";
      while (label_it != all_labels_.end ())
      {
        std::cout << *label_it << " ";
        label_it++;
      }
      std::cout << std::endl;

      // convert the tf transform to Eigen...
      Eigen::Affine3d hole_to_tabletop;
      tf::transformTFToEigen (tf_transform, hole_to_tabletop);

      // Transform the origin of the frame where the holes were recorded
      Eigen::Vector3d transformed_origin;
      pcl::transformPoint (Eigen::Vector3d::Zero (), transformed_origin, hole_to_tabletop);

      std::vector<LabelCloudPtr> current_holes;
      current_holes.reserve (holes->convex_hulls.size ());

      // transform all convex hulls point clouds into the table frame (aligned with x-y-plane)
      for (size_t i = 0; i < holes->convex_hulls.size (); ++i)
      {
        // convert sensor_msgs::PointCloud2 to PCLPointCloud2 to pcl::PointCloud<T>
        LabelCloudPtr hole_hull (new LabelCloud);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL (holes->convex_hulls[i], pcl_pc2);
        pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
        pcl::fromPCLPointCloud2 (pcl_pc2, *hole_hull);
        pcl::console::setVerbosityLevel (pcl::console::L_INFO);

        ROS_DEBUG ("finished conversion, hole hull size: %lu",
            hole_hull->points.size ());

        // ----- sample inside of hole -----

        // transform into table frame (tabletop aligned with x-y-plane)
        LabelCloudPtr xy_hole_hull (new LabelCloud);
        pcl::transformPointCloud (*hole_hull, *xy_hole_hull, hole_to_tabletop);

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
        transforms_.push_back (hole_to_tabletop);

        // transform sampled hole back into its original frame (where sensor is placed at (0,0,0))
        LabelCloudPtr hole_sample_cloud (new LabelCloud);
        pcl::transformPointCloud (*xy_hole_sample_cloud, *hole_sample_cloud, hole_to_tabletop.inverse ());

        // ----- create frustum -----
        LabelCloudPtr frustum (new LabelCloud);
        createSampleRays (hole_sample_cloud, frustum, leaf_size[0]);
        ROS_DEBUG ("created frustum, size: %lu", frustum->points.size ());
        frustum->header = hole_sample_cloud->header;

        if (frustum->points.size () > 0)
        {
          // add currently used label to the set of available labels
          available_labels_.insert (current_label);
          // transform frustum into global frame
          LabelCloudPtr transformed_frustum (new LabelCloud);
          pcl::transformPointCloud (*frustum, *transformed_frustum, hole_to_tabletop);

          // define reference bounding box from the first received frustum - this way
          // all octrees should be properly aligned
          if (!reference_bb_set_)
          {
            LabelPoint min_p, max_p;
            pcl::getMinMax3D (*transformed_frustum, min_p, max_p);
            min_ref_bb_ = Eigen::Vector3d (min_p.x, min_p.y, min_p.z);
            max_ref_bb_ = Eigen::Vector3d (max_p.x, max_p.y, max_p.z);
            reference_bb_set_ = true;
          }

          // create a downsampled version of the transformed frustum -- so that the overall
          // point cloud is less dense and checks of individual leafs become much faster
          // pcl::VoxelGrid unfortunately produces some strange gaps - therefore octree is used for voxelization
          LabelCloudPtr v_trans_frustum (new LabelCloud);
          LabelOctree::Ptr voxelize_tree (new LabelOctree (octree_resolution_));
          // set reference bounding box
          voxelize_tree->defineBoundingBox (min_ref_bb_[0], min_ref_bb_[1], min_ref_bb_[2],
          max_ref_bb_[0], max_ref_bb_[1], max_ref_bb_[2]);

          voxelize_tree->setInputCloud (transformed_frustum);
          voxelize_tree->addPointsFromInputCloud ();
          voxelize_tree->getOccupiedVoxelCenters (v_trans_frustum->points);

          // add the current label to the points representing leaf center points
          LabelCloud::VectorType::iterator v_trans_frustum_it = v_trans_frustum->points.begin ();
          while (v_trans_frustum_it != v_trans_frustum->points.end ())
          {
            v_trans_frustum_it->label = current_label;
            v_trans_frustum_it++;
          }

          // add frustum to collection of all frusta
          all_frusta_->points.reserve (all_frusta_->points.size () +
              transformed_frustum->points.size ());
          all_frusta_->points.insert (all_frusta_->points.end (),
              v_trans_frustum->points.begin (), v_trans_frustum->points.end ());
          // adapt dimensions of point cloud
          all_frusta_->width = all_frusta_->points.size ();
          all_frusta_->height = 1;
          ROS_DEBUG ("inserted frustum points into all_frusta_");
        }
      }
      // store all convex hulls of the current Holes msgs (aligned to tabletop)
      transformed_holes_.push_back (current_holes);

      ros::Time before_intersec_time = ros::Time::now ();
      // compute intersection
      this->computeIntersection ();
      ros::Time finished_intersec_time = ros::Time::now ();

      ros::Duration cb_duration = finished_intersec_time - cb_start_time;
      ros::Duration intersec_duration = finished_intersec_time - before_intersec_time;
      ROS_INFO ("callback duration: %lf", cb_duration.toSec ());
      ROS_INFO ("intersec time: %lf", intersec_duration.toSec ());


      ROS_INFO ("Finished callback, intersections and visualization for %lu views are computed", collected_views_.size ());
    };

    void computeIntersection (void)
    {
      // remove lingering contents of output clouds
      intersec_cloud_->points.clear ();

      if (available_labels_.size () < 1)
      {
        ROS_WARN ("called 'computeIntersection ()', but no label exists; Exiting intersection computation");
        return;
      }
      // clear old content from octree and intersection cloud
      octree_->deleteTree ();
      octree_->defineBoundingBox (min_ref_bb_[0], min_ref_bb_[1], min_ref_bb_[2],
          max_ref_bb_[0], max_ref_bb_[1], max_ref_bb_[2]);
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
      Eigen::Vector3d center_double;
      geometry_msgs::Point voxel_center;
      while (leaf_it != octree_->leaf_end ())
      {
        nr_leaves++;
        // remove old point indices
        leaf_point_indices->indices.clear ();

        // retrieve container for the current leaf
        LeafContainer &container = leaf_it.getLeafContainer ();
        container.getPointIndices (leaf_point_indices->indices);

        // check if enough points in leaf
        if (leaf_point_indices->indices.size () >= min_leaf_points_)
        {
          filled_leaves++;
          extract.setIndices (leaf_point_indices);
          extract.filter (*leaf_cloud);

          // retrieve the center point of the current leaf
          octree_->getVoxelBounds (leaf_it, min, max);
          center = min + max;
          center /= 2.0f;
          // transform center from tabletop to map frame
          center_double = Eigen::Vector3d (center[0], center[1], center[2]);
          center_double = table_to_map_transform_ * center_double;
          voxel_center.x = center_double[0];
          voxel_center.y = center_double[1];
          voxel_center.z = center_double[2];

          size_t nr_detected_labels;
          bool detected_fraction;
          size_t view_count;
          if (isLeafInIntersectionViewPoint (*leaf_cloud, view_count))
          {
            intersec_marker_.points.push_back (voxel_center);
            intersec_leaves++;
            intersec_cloud_->points.insert (intersec_cloud_->points.end (),
                leaf_cloud->points.begin (), leaf_cloud->points.end ());
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

        if (map_frame_.compare (tabletop_frame_) != 0)
        {
          // transform and publish in map frame
          LabelCloudPtr tmp_cloud (new LabelCloud);
          pcl::transformPointCloud (*intersec_cloud_, *tmp_cloud, table_to_map_transform_);
          header.frame_id = map_frame_;
          pcl_conversions::toPCL (header, tmp_cloud->header);

          // publish
          intersec_pub_.publish (tmp_cloud);
        }
        else
        {
          // publish
          intersec_pub_.publish (intersec_cloud_);
        }
      }
    };

    void publish_markers (void)
    {
      intersec_marker_.header.stamp = ros::Time::now ();
      non_intersec_marker_.header.stamp = ros::Time::now ();
      // set frame_id of markers to map frame
      intersec_marker_.header.frame_id = map_frame_;
      non_intersec_marker_.header.frame_id = map_frame_;

      // publish visualization marker
      visualization_msgs::MarkerArray vis_marker_array;
      vis_marker_array.markers.push_back (intersec_marker_);
      vis_marker_array.markers.push_back (non_intersec_marker_);
      vis_pub_.publish (vis_marker_array);

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

      if (map_frame_.compare (tabletop_frame_) != 0)
      {
        // copy currently existing markers
        visualization_msgs::MarkerArray frusta_marker_in_map_frame (frusta_marker_);
        // transform points into map frame and change header for each marker
        tf::Point p_in_tt_frame, p_in_map_frame;
        for (size_t i = 0; i < frusta_marker_in_map_frame.markers.size (); ++i)
        {
          frusta_marker_in_map_frame.markers[i].header.frame_id = map_frame_;
          std::vector<geometry_msgs::Point>::iterator p_it = frusta_marker_in_map_frame.markers[i].points.begin ();
          while (p_it != frusta_marker_in_map_frame.markers[i].points.end ())
          {
            tf::pointMsgToTF ((*p_it), p_in_tt_frame);
            p_in_map_frame = table_to_map_ (p_in_tt_frame);
            tf::pointTFToMsg (p_in_map_frame, (*p_it));
            p_it++;
          }
        }

        // publish marker in map frame
        all_frusta_pub_.publish (frusta_marker_in_map_frame);
      }
      else
      {
        // publish marker in map frame
        all_frusta_pub_.publish (frusta_marker_);
      }

      ROS_DEBUG ("published markers, currently %lu views collected", collected_views_.size ());
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
      vis_pub_.publish (clear_marker_array_);
      frusta_marker_.markers.clear ();
      frame_change_indices.clear ();
      // reset reference bounding box
      reference_bb_set_ = false;
      min_ref_bb_ = max_ref_bb_ = Eigen::Vector3d::Zero ();
      // ...and exit
      ROS_INFO ("Reset HoleIntersector");
      return true;
    };


  private:
    size_t min_leaf_points_;
    float octree_resolution_;
    std::string tabletop_frame_;
    std::string map_frame_;

    double current_yaw_;
    int angle_resolution_;
    int opening_angle_;
    int min_bin_marks_;

    bool reference_bb_set_;
    Eigen::Vector3d min_ref_bb_;
    Eigen::Vector3d max_ref_bb_;

    Eigen::Affine3d table_to_map_transform_;
    tf::StampedTransform table_to_map_;

    std::set<uint32_t> all_labels_;

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
      intersec_marker_.color.a = 1.0;
      intersec_marker_.color.r = 0.0;
      intersec_marker_.color.g = 1.0;
      intersec_marker_.color.b = 0.0;

      non_intersec_marker_ = visualization_msgs::Marker (intersec_marker_);
      non_intersec_marker_.ns = "non_intersec";
      non_intersec_marker_.id = 1;
      non_intersec_marker_.color.a = 0.15;
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
    ros::NodeHandle param_handle_;
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
        const std::set<uint32_t> &labels, size_t &nr_detected_labels)
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
      nr_detected_labels = 0;
      detected_label = leaf_labels.begin ();
      while (detected_label != leaf_labels.end ())
      {
        if (detected_label->second < min_points_per_label)
        {
          missing_label = true;
        }
        else
        {
          nr_detected_labels++;
        }
        detected_label++;
      }
      if (!missing_label)
        return true;
      return false;
    };

    bool isLeafInIntersectionViewPoint (const LabelCloud &leaf_cloud,
        size_t &view_count)
    {
      // check if number of points is sufficient
      if (leaf_cloud.points.size () < (min_bin_marks_ / opening_angle_))
      {
        return false;
      }
      // gather all labels in the point cloud
      std::set<uint32_t> leaf_labels;
      LabelCloud::VectorType::const_iterator p_it = leaf_cloud.points.begin ();
      while (p_it != leaf_cloud.points.end ())
      {
        leaf_labels.insert (p_it->label);
        p_it++;
      }

      // now generate the viewpoint marker array from the detected labels
      std::vector<uint8_t> viewpoint_marker (angle_resolution_, 0);
      std::set<uint32_t>::const_iterator label_it = leaf_labels.begin ();
      while (label_it != leaf_labels.end ())
      {
        for (int i = -opening_angle_; i <= opening_angle_; ++i)
        {
          viewpoint_marker[(*label_it + i + angle_resolution_) % angle_resolution_] = 1;
        }
        label_it++;
      }

      // gather the number of marks in the viewpoint marker
      view_count = 0;
      std::vector<uint8_t>::const_iterator marker_it = viewpoint_marker.begin ();
      while (marker_it != viewpoint_marker.end ())
      {
        view_count += *marker_it++;
      }
      if (view_count >= min_bin_marks_)
      {
        return true;
      }

      return false;
    };

    bool isLeafInIntersectionViewPoint2 (const LabelCloud &leaf_cloud,
        size_t &view_count)
    {
      // check if number of points is sufficient
      if (leaf_cloud.points.size () < (min_bin_marks_ / opening_angle_))
      {
        return false;
      }
      // gather all labels in the point cloud
      std::vector<uint32_t> leaf_labels (leaf_cloud.points.size ());
      for (size_t i = 0; i < leaf_cloud.points.size (); ++i)
      {
        leaf_labels[i] = leaf_cloud.points[i].label;
      }
      std::sort (leaf_labels.begin (), leaf_labels.end ());
      std::vector<uint32_t>::iterator last_label = std::unique (leaf_labels.begin (), leaf_labels.end ());

      // now generate the viewpoint marker array from the detected labels
      std::vector<bool> viewpoint_marker (angle_resolution_, false);
      std::vector<uint32_t>::const_iterator label_it = leaf_labels.begin ();
      while (label_it != last_label)
      {
        for (int i = -opening_angle_; i <= opening_angle_; ++i)
        {
          viewpoint_marker[(*label_it + i + angle_resolution_) % angle_resolution_] = true;
        }
        label_it++;
      }

      // gather the number of marks in the viewpoint marker
      view_count = 0;
      std::vector<bool>::const_iterator marker_it = viewpoint_marker.begin ();
      while (marker_it != viewpoint_marker.end ())
      {
        if (*marker_it++)
        {
          view_count++;
        }
      }
      if (view_count >= min_bin_marks_)
      {
        return true;
      }

      return false;
    };
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "HoleIntersector");

  HoleIntersector h (0.005f, 1, "tracked_table", "map");

  ros::spin ();

  return 0;
}
