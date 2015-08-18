#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <limits>

#include<transparent_object_reconstruction/Holes.h>
#include<transparent_object_reconstruction/tools.h>

struct HoleDetector
{
  static void calcPlaneTransformation (Eigen::Vector3f plane_normal,
      const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
  {
    Eigen::Vector3f ortho_to_normal = Eigen::Vector3f::Unit (3,1); // (0, 1, 0)

    float x, y, z;
    double angle = ::pcl::getAngle3D (Eigen::Vector4f (plane_normal[0],
          plane_normal[1], plane_normal[2], 0.0f),
        Eigen::Vector4f::Unit (4,2));
    if (fabs (angle - M_PI) < angle)
    {
      angle = fabs (angle - M_PI);
    }
    if (angle < ::pcl::deg2rad (.5f))
    {
      plane_normal.normalize ();
    }
    else
    {
      if (fabs (plane_normal[2]) > std::numeric_limits<float>::epsilon ())
      {
        x = y = 1.0f; // chosen arbitrarily
        z = plane_normal[0] + plane_normal[1];
        z /= -plane_normal[2];
      }
      else if (fabs (plane_normal[1]) > std::numeric_limits<float>::epsilon ())
      {
        x = z = 1.0f; // chosen arbitrarily
        y = plane_normal[0];
        y /= -plane_normal[1];
      }
      else
      {
        x = .0f;
        y = z = 1.0f; // chosen arbitrarily
      }
      ortho_to_normal = Eigen::Vector3f (x, y, z);
      ortho_to_normal.normalize ();
    }
    ::pcl::getTransformationFromTwoUnitVectorsAndOrigin (ortho_to_normal,
        plane_normal, origin, transformation);
  }

  template <typename PointT>
    void addRemoveIndices (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        const std::vector<Eigen::Vector2i> &convex_hull,
        ::pcl::PointIndices::Ptr &remove_indices)
    {
      // retrieve 2D bbox of convex hull
      Eigen::Vector2i min (cloud->width, cloud->height);
      Eigen::Vector2i max (0, 0);

      std::vector<Eigen::Vector2i>::const_iterator hull_it = convex_hull.begin ();
      while (hull_it != convex_hull.end ())
      {
        if ((*hull_it)[0] < min[0])
          min[0] = (*hull_it)[0];
        if ((*hull_it)[1] < min[1])
          min[1] = (*hull_it)[1];
        if ((*hull_it)[0] > max[0])
          max[0] = (*hull_it)[0];
        if ((*hull_it)[1] > max[1])
          max[1] = (*hull_it)[1];
        hull_it++;
      }

      // reserve the upper limit of newly needed entries into remove_indices
      remove_indices->indices.reserve (remove_indices->indices.size () + (max[0] - min[0]) * (max[1] - min[1]));
      Eigen::Vector2i query_point;
      int point_index;

      for (int u = min[0]; u < max[0]; ++u)
      {
        query_point[0] = u;
        for (int v = min[1]; v < max[1]; ++v)
        {
          query_point[1] = v;
          if (pointInPolygon2D (convex_hull, query_point))
          {
            if (::pcl::isFinite (cloud->at (u, v)))
            {
              convert2DCoordsToIndex (cloud, u, v, point_index);
              remove_indices->indices.push_back (point_index);
            }
          }
        }
      }
    }

  static bool pointInPolygon2D (const std::vector<Eigen::Vector2i> &polygon,
      const Eigen::Vector2i &query_point)
  {
    bool inside = false;

    std::vector<Eigen::Vector2i>::const_iterator start_it, end_it;
    start_it = polygon.end () - 1;  // last vertex
    end_it = polygon.begin ();
    bool start_above, end_above;

    start_above = (*start_it)[1] >= query_point[1] ? true: false;
    while (end_it != polygon.end ())
    {
      end_above = (*end_it)[1] >= query_point[1] ? true : false;

      if (start_above != end_above)
      {
        if (((*end_it)[1] - query_point[1]) * ((*end_it)[0] - (*start_it)[0]) <=
            ((*end_it)[1] - (*start_it)[1]) * ((*end_it)[0] - query_point[0]))
        {
          if (end_above)
          {
            inside = !inside;
          }
        }
        else
        {
          if (!end_above)
          {
            inside = !inside;
          }
        }
      }
      start_above = end_above;
      start_it = end_it;
      end_it++;
    }

    return inside;
  }

  template <typename PointT>
    static void recursiveNANGrowing (
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        int column, int row,
        std::vector<Eigen::Vector2i> &hole_2Dcoords,
        std::vector<Eigen::Vector2i> &border_2Dcoords,
        std::vector<std::vector<bool> > &visited)
    {
      // check dimension
      if (column < 0 || row < 0 ||
          column >= cloud->width || row >= cloud->height)
        return;
      // check if already visited and nan-point -- borders may be part of two neighboring regions
      auto p = cloud->at (column, row);
      if (visited[column][row] && !pcl_isfinite (p.x))
        return;
      visited[column][row] = true;
      Eigen::Vector2i coordinates (column, row);
      if (pcl_isfinite (p.x))
      {
        border_2Dcoords.push_back (coordinates);
        return;
      }
      hole_2Dcoords.push_back (coordinates);
      recursiveNANGrowing (cloud, column - 1, row, hole_2Dcoords, border_2Dcoords, visited);
      recursiveNANGrowing (cloud, column + 1, row, hole_2Dcoords, border_2Dcoords, visited);
      recursiveNANGrowing (cloud, column, row - 1, hole_2Dcoords, border_2Dcoords, visited);
      recursiveNANGrowing (cloud, column, row + 1, hole_2Dcoords, border_2Dcoords, visited);
    }

  template <typename PointT>
    static bool convertIndexTo2DCoords (
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        int index, int &x_coord, int &y_coord)
    {
      if (!cloud->isOrganized ())
      {
        x_coord = y_coord = -1;
        std::cerr << "convertIndexTo2DCoords: cloud needs to be organized" << std::endl;
        return false;
      }
      if (index < 0 || index >= cloud->points.size ())
      {
        x_coord = y_coord = -1;
        std::cerr << "convertIndexTo2DCoords: specified point index " << index << " invalid" << std::endl;
        return false;
      }
      y_coord = index / cloud->width;
      x_coord = index - cloud->width * y_coord;

      return true;
    }

  template <typename PointT>
    static bool convert2DCoordsToIndex (
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        int x_coord, int y_coord, int &index)
    {
      if (x_coord < 0 || y_coord < 0)
      {
        std::cerr << "Image coordinates need to be larger than 0" << std::endl;
        return false;
      }
      if (!cloud->isOrganized () && y_coord > 1)
      {
        std::cerr << "Cloud not organized, but y_coord > 1" << std::endl;
        return false;
      }
      if (x_coord >= cloud->width || y_coord >= cloud->height)
      {
        std::cerr << "2D coordinates (" << x_coord << ", " << y_coord << ") outside"
          << " of cloud dimension " << cloud->width << "x" << cloud->height << std::endl;
        return false;
      }
      index = y_coord * cloud->width + x_coord;

      return true;
    }

  template <typename PointT>
    static void getBoundingBox2DConvexHull (
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        const ::pcl::PointIndices &hull_indices,
        Eigen::Vector2i &min, Eigen::Vector2i &max,
        std::vector<Eigen::Vector2i> &hull_2Dcoords)
    {
      // initialize output
      max[0] = max[1] = 0;
      min[0] = cloud->width;
      min[1] = cloud->height;

      hull_2Dcoords.clear ();
      hull_2Dcoords.reserve (hull_indices.indices.size ());

      int u, v;

      std::vector<int>::const_iterator index_it = hull_indices.indices.begin ();
      while (index_it != hull_indices.indices.end ())
      {
        if (convertIndexTo2DCoords (cloud, *index_it++, u, v))
        {
          hull_2Dcoords.push_back (Eigen::Vector2i (u, v));
          if (u < min[0])
            min[0] = u;
          if (v < min[1])
            min[1] = v;
          if (u > max[0])
            max[0] = u;
          if (v > max[1])
            max[1] = v;
        }
      }
    }

  static void declare_params (tendrils& params)
  {
    params.declare<size_t> ("min_hole_size", "Minimal numbers of connected pixels in the depth image to form a hole", 15);
    params.declare<float> ("inside_out_factor", "Determines if a nan-region is outside of table hull (#outside > #points_inside * inside_out_factor)", 2.0f);
    params.declare<float> ("plane_dist_threshold", "Distance threshold for plane classification", .02f);
  }

  static void declare_io ( const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<::pcl::PointIndices::ConstPtr> ("hull_indices", "The indices describing the convex hull to the table surface.");
    inputs.declare<::pcl::ModelCoefficients::ConstPtr> ("model", "Model coefficients for the planar table surface.");
    outputs.declare<ecto::pcl::PointCloud> ("output", "Filtered Cloud.");
    outputs.declare<transparent_object_reconstruction::Holes::ConstPtr> ("holes", "Detected holes inside the table convex hull.");
    outputs.declare<::pcl::PointIndices::ConstPtr> ("remove_indices", "Indices of points inside the detected holes.");
  }

  void configure( const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    min_hole_size_ = params["min_hole_size"];
    inside_out_factor_ = params["inside_out_factor"];
    plane_dist_threshold_ = params["plane_dist_threshold"];
    hull_indices_ = inputs["hull_indices"];
    model_ = inputs["model"];
    output_ = outputs["output"];
    holes_mgs_ = outputs["holes"];
    remove_indices_ = outputs["remove_indices"];
  }

  template <typename PointT>
    int process( const tendrils& inputs, const tendrils& outputs,
        boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input)
    {
      Eigen::Affine3f trans;
      Eigen::Affine3f back_trans;
      Eigen::Vector3f center = Eigen::Vector3f::Zero ();
      Eigen::Vector3f plane_normal = Eigen::Vector3f ((*model_)->values[0],
          (*model_)->values[1], (*model_)->values[2]);
      std::vector<Eigen::Vector3f> min_points (3);
      std::vector<Eigen::Vector3f> max_points (3);
      for (size_t i = 0; i < 3; ++i)
      {
        min_points[i] = Eigen::Vector3f (std::numeric_limits<float>::max (),
            std::numeric_limits<float>::max (),
            std::numeric_limits<float>::max ());
        max_points[i] = Eigen::Vector3f (-std::numeric_limits<float>::max (),
            -std::numeric_limits<float>::max (),
            -std::numeric_limits<float>::max ());
      }
      auto inlier_it = input->points.begin ();
      while (inlier_it != input->points.end ())
      {
        if (inlier_it-> x < min_points[0][0])
        {
          min_points[0] = Eigen::Vector3f (inlier_it->x, inlier_it->y, inlier_it->z);
        }
        if (inlier_it-> x < min_points[1][1])
        {
          min_points[1] = Eigen::Vector3f (inlier_it->x, inlier_it->y, inlier_it->z);
        }
        if (inlier_it-> x < min_points[2][2])
        {
          min_points[2] = Eigen::Vector3f (inlier_it->x, inlier_it->y, inlier_it->z);
        }
        if (inlier_it->x > max_points[0][0])
        {
          max_points[0] = Eigen::Vector3f (inlier_it->x, inlier_it->y, inlier_it->z);
        }
        if (inlier_it->y > max_points[1][1])
        {
          max_points[1] = Eigen::Vector3f (inlier_it->x, inlier_it->y, inlier_it->z);
        }
        if (inlier_it->z > max_points[2][2])
        {
          max_points[2] = Eigen::Vector3f (inlier_it->x, inlier_it->y, inlier_it->z);
        }
        inlier_it++;
      }
      for (size_t i = 0; i < 3; ++i)
      {
        center[0] += min_points[i][0] + max_points[i][0];
        center[1] += min_points[i][1] + max_points[i][1];
        center[2] += min_points[i][2] + max_points[i][2];
      }
      center /= 6.0f;

      calcPlaneTransformation (plane_normal, center, trans);
      back_trans = trans.inverse ();

      Eigen::Vector2i table_min, table_max;
      std::vector<Eigen::Vector2i> hull_2Dcoords;
      std::vector<std::vector<Eigen::Vector2i> > all_hole_2Dcoords;
      std::vector<std::vector<Eigen::Vector2i> > all_border_2Dcoords;
      getBoundingBox2DConvexHull (input, **hull_indices_, table_min, table_max, hull_2Dcoords);

      std::vector<std::vector<bool> > visited (input->width, std::vector<bool> (input->height, false));

      for (int u = table_min[0]; u < table_max[0]; ++u)
      {
        for (int v = table_min[1]; v < table_max[1]; ++v)
        {
          if (!visited[u][v] &&                                             // already used?
              pointInPolygon2D (hull_2Dcoords, Eigen::Vector2i (u, v)) &&   // inside hull?
              !(pcl::isFinite (input->at (u,v))))                           // nan point?
          {
            std::vector<Eigen::Vector2i> hole_2Dcoords;
            std::vector<Eigen::Vector2i> border_2Dcoords;
            recursiveNANGrowing (input, u, v, hole_2Dcoords, border_2Dcoords, visited);
            if (hole_2Dcoords.size () > *min_hole_size_)
            {
              all_hole_2Dcoords.push_back (hole_2Dcoords);
              all_border_2Dcoords.push_back (border_2Dcoords);
            }
          }
        }
      }

      // collected all nan-regions that contain at least 1 nan-pixel inside the convex hull of the table
      std::vector<std::vector<Eigen::Vector2i> >::const_iterator all_holes_it;
      std::vector<std::vector<Eigen::Vector2i> >::const_iterator all_borders_it;
      std::vector<Eigen::Vector2i>::const_iterator hole_it;
      all_holes_it = all_hole_2Dcoords.begin ();
      all_borders_it = all_border_2Dcoords.begin ();

      std::vector<std::vector<Eigen::Vector2i> > outside_holes;
      std::vector<std::vector<Eigen::Vector2i> > outside_borders;
      std::vector<std::vector<Eigen::Vector2i> > overlap_holes;
      std::vector<std::vector<Eigen::Vector2i> > overlap_borders;
      std::vector<std::vector<Eigen::Vector2i> > inside_holes;
      std::vector<std::vector<Eigen::Vector2i> > inside_borders;

      outside_holes.reserve (all_hole_2Dcoords.size ());
      outside_borders.reserve (all_hole_2Dcoords.size ());
      overlap_holes.reserve (all_hole_2Dcoords.size ());
      overlap_borders.reserve (all_hole_2Dcoords.size ());
      inside_holes.reserve (all_hole_2Dcoords.size ());
      inside_borders.reserve (all_hole_2Dcoords.size ());

      size_t inside, outside;

      while (all_holes_it != all_hole_2Dcoords.end ())
      {
        inside = outside = 0;
        hole_it = all_holes_it->begin ();
        while (hole_it != all_holes_it->end ())
        {
          if (pointInPolygon2D (hull_2Dcoords, *hole_it))
          {
            inside++;
          }
          else
          {
            outside++;
          }
          hole_it++;
        }
        // TODO: probably some fraction dependent on min_hole_size_ should be used
        if (outside > 0)
        {
          if (outside > inside * (*inside_out_factor_))
          {
            // TODO: could we use these later somewhere or can they be safely discarded?
            outside_holes.push_back (*all_holes_it);
            outside_borders.push_back (*all_borders_it);
          }
          else
          {
            overlap_holes.push_back (*all_holes_it);
            overlap_borders.push_back (*all_borders_it);
          }
        }
        else
        {
          inside_holes.push_back (*all_holes_it);
          inside_borders.push_back (*all_borders_it);
        }
        all_holes_it++;
        all_borders_it++;
      }
      // create array to store all non nan-points that are enclosed by the hole
      // and should be removed before clustering
      ::pcl::PointIndices::Ptr remove_indices (new ::pcl::PointIndices);
      std::vector<Eigen::Vector2i>::const_iterator coord_it;

      Eigen::Vector4f plane_coefficients ((*model_)->values[0], (*model_)->values[1],
          (*model_)->values[2], (*model_)->values[3]);

      // create representations for the holes in the plane (atm only for complete hulls)
      auto holes_msg= boost::make_shared<transparent_object_reconstruction::Holes>();
      holes_msg->convex_hulls.reserve (inside_holes.size ());
      for (size_t i = 0; i < inside_holes.size (); ++i)
      {
        auto border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        border_cloud->points.reserve (inside_borders[i].size ());
        coord_it = inside_borders[i].begin ();
        while (coord_it != inside_borders[i].end ())
        {
          border_cloud->points.push_back (input->at ((*coord_it)[0], (*coord_it)[1]));
          coord_it++;
        }
        // set dimensions of border cloud
        border_cloud->width = border_cloud->points.size ();
        border_cloud->height = 1;

        double dist_sum = .0f;
        auto border_it = border_cloud->points.begin ();

        while (border_it != border_cloud->points.end ())
        {
          dist_sum += ::pcl::pointToPlaneDistance (*border_it++, plane_coefficients);
        }

        double avg_dist = dist_sum / static_cast<double> (border_cloud->points.size ());
        if (avg_dist > *plane_dist_threshold_)
        {
          // skip hole! should something more be done?
          continue;
        }

        // project border into plane
        auto proj_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        typename ::pcl::ProjectInliers<PointT> proj_border;
        proj_border.setInputCloud (border_cloud);
        proj_border.setModelType (pcl::SACMODEL_PLANE);
        proj_border.setModelCoefficients (*model_);
        proj_border.filter (*proj_border_cloud);

        // compute the convex hull of the (projected) border
        auto conv_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        ::pcl::PointIndices conv_border_indices;
        typename ::pcl::ConvexHull<PointT> c_hull;
        c_hull.setInputCloud (proj_border_cloud);
        c_hull.setDimension (2);
        c_hull.reconstruct (*conv_border_cloud);
        c_hull.getHullPointIndices (conv_border_indices);

        std::vector<Eigen::Vector2i> convex_hull_polygon;
        convex_hull_polygon.reserve (conv_border_indices.indices.size ());
        std::vector<int>::const_iterator hull_it = conv_border_indices.indices.begin ();
        while (hull_it != conv_border_indices.indices.end ())
        {
          convex_hull_polygon.push_back (inside_borders[i][*hull_it++]);
        }
        // store indices of points that are inside the convex hull - these will be removed later
        addRemoveIndices (input, convex_hull_polygon, remove_indices);

        // add current hole to Hole message
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg (*conv_border_cloud, pc2);
        // TODO: check if header information is copied as well (and if it is set in the first palce)
        holes_msg->convex_hulls.push_back (pc2);
      }
      // work on the holes that are partially inside the convex hull
      for (size_t i = 0; i < overlap_borders.size (); ++i)
      {
        unsigned int inside_points = 0;
        double dist_sum = 0.0f;
        coord_it = overlap_borders[i].begin ();
        // check for alignment of the points that are considered in the convex hull
        while (coord_it != overlap_borders[i].end ())
        {
          if (pointInPolygon2D (hull_2Dcoords, *coord_it))
          {
            dist_sum += ::pcl::pointToPlaneDistance (input->at ((*coord_it)[0], (*coord_it)[1]), plane_coefficients);
            inside_points++;
          }
          coord_it++;
        }
        double avg_dist = dist_sum / static_cast<double> (inside_points);
        if (avg_dist > *plane_dist_threshold_ * 3.0f) // TODO: perhaps remove the points with the largest 3 distances instead?
        {
          //skip hole! should something more be done?
          continue;
        }
        auto border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        border_cloud->points.reserve (overlap_borders[i].size ());
        coord_it = overlap_borders[i].begin ();
        PointT border_p, projection;
        while (coord_it != overlap_borders[i].end ())
        {
          border_p = input->at ((*coord_it)[0], (*coord_it)[1]);
          if (::pcl::pointToPlaneDistance (border_p, plane_coefficients) > *plane_dist_threshold_ * 2.0f)
          {
            // compute perspective projection of border point onto plane
            if (projectPointOnPlane2<PointT> (border_p, projection, plane_coefficients))
            {
              // if a projection exists, add it to the border cloud
              border_cloud->points.push_back (projection);
            }
            // otherwise discard this point
          }
          else
          {
            border_cloud->points.push_back (border_p);
          }
          coord_it++;
        }
        if (border_cloud->points.size () > 0)
        {
          // set dimensions of border cloud
          border_cloud->width = border_cloud->points.size ();
          border_cloud->height = 1;
          // make sure that all border points are in the plane
          auto proj_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
          typename ::pcl::ProjectInliers<PointT> proj_border;
          proj_border.setInputCloud (border_cloud);
          proj_border.setModelType (::pcl::SACMODEL_PLANE);
          proj_border.setModelCoefficients (*model_);
          proj_border.filter (*proj_border_cloud);

          // compute convex hull of projected border
          auto conv_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
          ::pcl::PointIndices conv_border_indices;
          typename ::pcl::ConvexHull<PointT> c_hull;
          c_hull.setInputCloud (proj_border_cloud);
          c_hull.setDimension (2);
          c_hull.reconstruct (*conv_border_cloud);
          c_hull.getHullPointIndices (conv_border_indices);

          std::vector<Eigen::Vector2i> convex_hull_polygon;
          convex_hull_polygon.reserve (conv_border_indices.indices.size ());
          std::vector<int>::const_iterator hull_it = conv_border_indices.indices.begin ();
          while (hull_it != conv_border_indices.indices.end ())
          {
            convex_hull_polygon.push_back (inside_borders[i][*hull_it++]);
          }
          // store indices of points that are inside the convex hull - these will be removed later
          addRemoveIndices (input, convex_hull_polygon, remove_indices);

          // add current hole to Hole message
          sensor_msgs::PointCloud2 pc2;
          pcl::toROSMsg (*conv_border_cloud, pc2);
          // TODO: check if header information is copied as well (and if it is set in the first palce)
          holes_msg->convex_hulls.push_back (pc2);
        }

      }

      if (remove_indices->indices.size () > 1)
      {
        // depending on what we want to do later it might be more convenient, if the remove indices are ordered
        ::std::sort (remove_indices->indices.begin (), remove_indices->indices.end ());
        // copy indices into output (and make sure that no index is inserted more than once
        ::pcl::PointIndices::Ptr tmp_indices (new ::pcl::PointIndices);
        tmp_indices->indices.reserve (remove_indices->indices.size ());
        std::vector<int>::const_iterator index_it = remove_indices->indices.begin ();
        tmp_indices->indices.push_back (*index_it++);
        while (index_it != remove_indices->indices.end ())
        {
          if (*index_it != tmp_indices->indices.back ())
          {
            tmp_indices->indices.push_back (*index_it);
          }
          index_it++;
        }
        *remove_indices_ = tmp_indices;
      }
      else
      {
        *remove_indices_ = remove_indices;
      }

      // set all points in the point cloud to nan, if their index was contained in remove_indices
      typename ::pcl::ExtractIndices<PointT> extractor;
      auto filtered_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
      extractor.setKeepOrganized (true);
      extractor.setNegative (true);
      extractor.setInputCloud (input);
      extractor.setIndices (remove_indices);
      extractor.filter (*filtered_cloud);

      *output_ = ecto::pcl::xyz_cloud_variant_t (filtered_cloud);
      *holes_mgs_ = holes_msg;

      return ecto::OK;
    }

  ecto::spore<size_t> min_hole_size_;
  ecto::spore<float> inside_out_factor_;
  ecto::spore<float> plane_dist_threshold_;
  ecto::spore<::pcl::PointIndices::ConstPtr> hull_indices_;
  ecto::spore<::pcl::ModelCoefficients::ConstPtr> model_; //TODO: is this what I get from the segmentation?
  ecto::spore<ecto::pcl::PointCloud> output_;
  ecto::spore<transparent_object_reconstruction::Holes::ConstPtr> holes_mgs_;
  ecto::spore<::pcl::PointIndices::ConstPtr> remove_indices_;
};

ECTO_CELL(hole_detection, ecto::pcl::PclCell<HoleDetector>,
    "HoleDetector", "Extract a new cloud given an existing cloud and a set of indices to extract.");

