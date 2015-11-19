#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <ros/console.h>

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
#include <vector>
#include <queue>
#include <set>

#include<transparent_object_reconstruction/Holes.h>
#include<transparent_object_reconstruction/tools.h>


struct Vector2iComp {
    bool operator() (const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) const
    {
      if (lhs[0] != rhs[0])
        return lhs[0] < rhs[0];
      return lhs[1] < rhs[1];
    }
};

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
    bool validPoint (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        const Eigen::Vector2i &coords)
    {
      if (coords[0] >= 0 && coords[1] >= 0 && coords[0] < cloud->width && coords[1] < cloud->height)
      {
        if (::pcl::isFinite<PointT> (cloud->at (coords[0], coords[1])))
          return true;
      }
      return false;
    }

  /**
    * @brief: Erodes a given border.
    * The 2D border coordinates are smeared into x-y direction by the specified erode size.
    * In the returned boorder coordinates, each coordinate is included only once.
    *
    * @param[in] input_cloud The complete point cloud
    * @param[in,out] border_coords The 2D coordinates of the border of the hole
    * @param[out] border_cloud The 3D points corresponding to the border of the hole
    * @param[in] erode_size The number of coordinates to extend the border
    */
  template <typename PointT>
  void erodeBorder (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &input_cloud,
      std::vector<Eigen::Vector2i> &border_coords,
      boost::shared_ptr<::pcl::PointCloud<PointT> > &border_cloud,
      size_t erode_size)
  {
    // clear / prepare outputs
    border_cloud->points.clear ();
    std::vector<Eigen::Vector2i> tmp_coords;

    Eigen::Vector2i tmp;
    Eigen::Vector2i x_shift (1, 0);
    Eigen::Vector2i y_shift = Eigen::Vector2i (0, 1);
    std::pair<std::set<Eigen::Vector2i, Vector2iComp>::iterator, bool> insert_res;
    std::set<Eigen::Vector2i, Vector2iComp> extended_border_coord_set;
    std::set<Eigen::Vector2i, Vector2iComp>::const_iterator set_it;
    std::vector<Eigen::Vector2i>::const_iterator c_it = border_coords.begin ();

    // do the erosion, but only add  coords that are valid and only add them once
    while (c_it != border_coords.end ())
    {
      extended_border_coord_set.insert (*c_it);
      for (size_t i = 1; i <= erode_size; ++i)
      {
        if (validPoint (input_cloud, (*c_it) + (i * x_shift)))
          extended_border_coord_set.insert ((*c_it) + (i * x_shift));
        if (validPoint (input_cloud, (*c_it) - (i * x_shift)))
          extended_border_coord_set.insert ((*c_it) - (i * x_shift));
        if (validPoint (input_cloud, (*c_it) + (i * y_shift)))
          extended_border_coord_set.insert ((*c_it) + (i * y_shift));
        if (validPoint (input_cloud, (*c_it) - (i * y_shift)))
          extended_border_coord_set.insert ((*c_it) - (i * y_shift));
      }
      c_it++;
    }
    border_cloud->points.reserve (extended_border_coord_set.size ());
    tmp_coords.reserve (extended_border_coord_set.size ());
    set_it = extended_border_coord_set.begin ();
    while (set_it != extended_border_coord_set.end ())
    {
      border_cloud->points.push_back (input_cloud->at ((*set_it)[0], (*set_it)[1]));
      tmp_coords.push_back (*set_it++);
    }
    border_cloud->header = input_cloud->header;
    border_cloud->width = border_cloud->points.size ();
    border_cloud->height = 1;
    border_coords.swap (tmp_coords);  // put the new coordinates into output argument
  }

  template <typename PointT>
    void projectBorderAndCreateHull (const boost::shared_ptr<::pcl::PointCloud<PointT> > &border_cloud,
        boost::shared_ptr<::pcl::PointCloud<PointT> > &convex_hull, ::pcl::PointIndices &convex_hull_indices)
    {
      auto proj_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();

      Eigen::Vector4f plane = Eigen::Vector4f ((*model_)->values[0],
          (*model_)->values[1], (*model_)->values[2], (*model_)->values[3]);

      projectPointCloudOnPlane<PointT> (border_cloud, plane, proj_border_cloud);

      // compute the convex hull of the (projected) border and retrieve the point indices
      typename ::pcl::ConvexHull<PointT> c_hull;
      c_hull.setInputCloud (proj_border_cloud);
      c_hull.setDimension (2);
      c_hull.reconstruct (*convex_hull);
      c_hull.getHullPointIndices (convex_hull_indices);
    }

  // TODO: more sophisticated test could check if the hull only touches the border (with on point)
  // or if there are consecutive points that touch the same border
  template <typename PointT>
    void get2DHullBBox (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        const std::vector<Eigen::Vector2i> &convex_hull,
        Eigen::Vector2i &min_bbox, Eigen::Vector2i &max_bbox, bool &touches_border)
    {
      min_bbox = Eigen::Vector2i (cloud->width, cloud->height);
      max_bbox = Eigen::Vector2i::Zero ();

      std::vector<Eigen::Vector2i>::const_iterator hull_it = convex_hull.begin ();
      while (hull_it != convex_hull.end ())
      {
        if ((*hull_it)[0] < min_bbox[0])
          min_bbox[0] = (*hull_it)[0];
        if ((*hull_it)[1] < min_bbox[1])
          min_bbox[1] = (*hull_it)[1];
        if ((*hull_it)[0] > max_bbox[0])
          max_bbox[0] = (*hull_it)[0];
        if ((*hull_it)[1] > max_bbox[1])
          max_bbox[1] = (*hull_it)[1];
        hull_it++;
      }

      if (min_bbox[0] == 0 || min_bbox[1] == 0 ||
          max_bbox[0] == cloud->width - 1 || max_bbox[1] == cloud->height - 1)
      {
        touches_border = true;
      }
      else
      {
        touches_border = false;
      }
    }

  float holeBorderLowerBoundDist2 (const Eigen::Vector4f &min_bbox_a, const Eigen::Vector4f &max_bbox_a,
      const Eigen::Vector4f &min_bbox_b, const Eigen::Vector4f &max_bbox_b)
  {
    Eigen::Vector3f min_dist = Eigen::Vector3f::Zero ();
    for (size_t i = 0; i < 3; ++i)
    {
      if (max_bbox_b[i] < min_bbox_a[i])
      {
        min_dist[i] = min_bbox_a[i] - max_bbox_b[i];
      }
      else if (max_bbox_a[i] < min_bbox_b[i])
      {
        min_dist[i] = min_bbox_b[i] - max_bbox_a[i];
      }
    }
    return min_dist.dot (min_dist);
  }

  template <typename PointT>
    float holeBorderLowerBoundDist2 (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &hole_border_a,
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &hole_border_b)
    {
      Eigen::Vector4f min_a, min_b, max_a, max_b;
      ::pcl::getMinMax3D<PointT> (*hole_border_a, min_a, max_a);
      ::pcl::getMinMax3D<PointT> (*hole_border_b, min_b, max_b);

      return holeBorderLowerBoundDist2 (min_a, max_a, min_b, max_b);
    }


  template <typename PointT>
    float minHoleBorderDist (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &hole_border_a,
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &hole_border_b)
    {
      typename ::pcl::PointCloud<PointT>::VectorType::const_iterator p_it = hole_border_a->points.begin ();
      typename ::pcl::PointCloud<PointT>::VectorType::const_iterator line_end_it;
      PointT line_start;
      float min_dist = std::numeric_limits<float>::max ();
      float dist_to_line;

      while (p_it != hole_border_a->points.end ())
      {
        line_start = hole_border_b->points.back ();
        line_end_it = hole_border_b->points.begin ();
        while (line_end_it != hole_border_b->points.end ())
        {
          dist_to_line = lineSegmentToPointDistance (line_start, *line_end_it, *p_it);
          if (dist_to_line < min_dist)
          {
            min_dist = dist_to_line;
          }
          line_start = *line_end_it;
          line_end_it++;
        }
        p_it++;
      }
      return min_dist;
    }


  template <typename PointT>
    void addRemoveIndices (boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        const std::vector<Eigen::Vector2i> &convex_hull,
        ::pcl::PointIndices::Ptr &remove_indices, bool &touches_border)
    {
      // retrieve 2D bbox of convex hull
      Eigen::Vector2i min, max;
      get2DHullBBox (cloud, convex_hull, min, max, touches_border);

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
          // check if point is inside the convex hull
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
    static void iterativeNANGrowing (
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > &cloud,
        const std::vector<Eigen::Vector2i> &hull_2Dcoords,
        const Eigen::Vector2i &table_min,
        const Eigen::Vector2i &table_max,
        std::vector<std::vector<Eigen::Vector2i> > &hole_2Dcoords,
        std::vector<std::vector<Eigen::Vector2i> > &all_border_2Dcoords)
    {
      // iterative version of the recursive NAN-growing that can segfault, due to recursion depth...
      // clear output arguments
      hole_2Dcoords.clear ();
      all_border_2Dcoords.clear ();

      // TODO: probably these should not be local variables, but also used as output arguments...
      // use local lists to simplify / speed up deletion of regions during merging
      std::list<std::set<Eigen::Vector2i, Vector2iComp> > borders_list;
      std::list<std::vector<Eigen::Vector2i> > holes_list;

      // create the inserted list locally
      std::vector<std::vector<bool> > inserted (cloud->width, std::vector<bool> (cloud->height, false));
      std::queue<Eigen::Vector2i> expansion_queue;

      // needed iterators
      std::list<std::set<Eigen::Vector2i, Vector2iComp> >::iterator border_list_it;
      std::list<std::vector<Eigen::Vector2i> >::iterator hole_list_it;


      // iterate over the bounding box of the table and check for seeds of nan regions
      for (int u = table_min[0]; u <= table_max[0]; ++u)
      {
        for (int v = table_min[1]; v <= table_max[1]; ++v)
        {
          if (!inserted[u][v] &&                                            // already used?
              !(::pcl::isFinite (cloud->at (u, v))) &&                      // nan point?
              pointInPolygon2D (hull_2Dcoords, Eigen::Vector2i (u, v)))     // inside hull?
          {
            std::vector<Eigen::Vector2i> current_hole;
            std::set<Eigen::Vector2i, Vector2iComp> current_border;

            // insert into expansion queue and mark as inserted
            expansion_queue.push (Eigen::Vector2i (u, v));
            inserted[u][v] = true;
            while (!expansion_queue.empty ())
            {
              // take the first element and remove it from the queue
              Eigen::Vector2i current_coordinate = expansion_queue.front ();
              expansion_queue.pop ();

              current_hole.push_back (current_coordinate);
              // generate potential neighbors;
              std::vector<Eigen::Vector2i> neighbors (4, current_coordinate);
              neighbors[0][0] -= 1;
              neighbors[1][0] += 1;
              neighbors[2][1] -= 1;
              neighbors[3][1] += 1;

              for (size_t i = 0; i < neighbors.size (); ++i)
              {
                Eigen::Vector2i neighbor = neighbors[i];
                // TODO: use this place to mark region as touching border?
                if ((neighbor[0] >= 0) && (neighbor[0] < cloud->width) &&
                    (neighbor[1] >= 0) && (neighbor[1] < cloud->height))
                {
                  if (::pcl::isFinite (cloud->at (neighbor[0], neighbor[1])))
                  {
                    // note that a border is not set as inserted
                    // since it could potentially be border of 2 neighboring regions
                    current_border.insert (neighbor);
                  }
                  else
                  {
                    if (!inserted[neighbor[0]][neighbor[1]])
                    {
                      // add nan coordinate to the queue and mark as inserted
                      expansion_queue.push (neighbor);
                      inserted[neighbor[0]][neighbor[1]] = true;
                    }
                  }
                }
              }
            }

            // check if the current region shares a border with an already existing region
            std::list<std::list<std::set<Eigen::Vector2i, Vector2iComp> >::iterator> shared_border_list;
            std::list<std::list<std::vector<Eigen::Vector2i> >::iterator> shared_hole_list;

            border_list_it = borders_list.begin ();
            hole_list_it = holes_list.begin ();

            while (border_list_it != borders_list.end ())
            {
              std::set<Eigen::Vector2i, Vector2iComp>::const_iterator border_it = current_border.begin ();
              while (border_it != current_border.end ())
              {
                if (border_list_it->find (*border_it) != border_list_it->end ())
                {
                  // add iterators at the front of the list with shared regions / border
                  shared_border_list.push_front (border_list_it);
                  shared_hole_list.push_front (hole_list_it);
                  break;  // we already know border is shared - no need to check further
                }
                border_it++;
              }
              border_list_it++;
              hole_list_it++;
            }

            // new region doesn't border any other region
            if (shared_border_list.size () == 0)
            {
              // insert new border / region at the end of existing lists
              holes_list.push_back (current_hole);
              borders_list.push_back (current_border);
            }
            else
            {
              // TODO: might look somewhat ugly, but is there a nicer way to do the same?
              // add current region / border to first detected neighbor (last element in shared lists)
              std::list<std::list<std::set<Eigen::Vector2i, Vector2iComp> >::iterator>::const_iterator merge_border_it;
              std::list<std::list<std::vector<Eigen::Vector2i> >::iterator>::const_iterator merge_hole_it;

              merge_border_it = shared_border_list.end ();
              merge_hole_it = shared_hole_list.end ();
              merge_border_it--;
              merge_hole_it--;

              (*merge_border_it)->insert (current_border.begin (), current_border.end ());
              (*merge_hole_it)->insert ((*merge_hole_it)->end (), current_hole.begin (), current_hole.end ());

              // do we need to merge some other regions as well?
              if (shared_border_list.size () > 1)
              {
                std::list<std::list<std::set<Eigen::Vector2i, Vector2iComp> >::iterator>::const_iterator shared_b_it;
                std::list<std::list<std::vector<Eigen::Vector2i> >::iterator>::const_iterator shared_h_it;
                shared_b_it = shared_border_list.begin ();
                shared_h_it = shared_hole_list.begin ();

                while (shared_b_it != merge_border_it)
                {
                  // add to border / hole to merge border / region
                  (*merge_border_it)->insert ((*shared_b_it)->begin (), (*shared_b_it)->end ());
                  (*merge_hole_it)->insert ((*merge_hole_it)->end (), (*shared_h_it)->begin (),
                      (*shared_h_it)->end ());
                  // delete old border / region
                  borders_list.erase ((*shared_b_it));
                  holes_list.erase ((*shared_h_it));
                  // advance to next border / region to be merged
                  shared_b_it++;
                  shared_h_it++;
                }
              }
            }
          }
        }
      }

      // TODO: change signature of method to provide lists as output arguments, since we might want to delete some regions later on...
      all_border_2Dcoords.reserve (borders_list.size ());
      hole_2Dcoords.reserve (holes_list.size ());

      border_list_it = borders_list.begin ();
      hole_list_it = holes_list.begin ();
      while (border_list_it != borders_list.end ())
      {
        all_border_2Dcoords.push_back (std::vector<Eigen::Vector2i> (border_list_it->begin (),
              border_list_it->end ()));
        hole_2Dcoords.push_back (*hole_list_it);
        border_list_it++;
        hole_list_it++;
      }
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

  template <typename PointT>
    bool minDistAboveThreshold (const boost::shared_ptr<::pcl::PointCloud<PointT> > &table_convex_hull,
        const boost::shared_ptr<::pcl::PointCloud<PointT> > &inside_table_border,
        float threshold)
    {
      typename ::pcl::PointCloud<PointT>::VectorType::const_iterator p_it = inside_table_border->points.begin ();
      typename ::pcl::PointCloud<PointT>::VectorType::const_iterator table_hull_it;
      float max_min_dist = -std::numeric_limits<float>::max ();
      PointT line_point;
      float dist_to_line;
      // determine for each point of the hole border inside the hull the closest line of the convex hull
      while (p_it != inside_table_border->points.end ())
      {
        float min_dist = std::numeric_limits<float>::max ();
        line_point = table_convex_hull->points.back ();
        table_hull_it = table_convex_hull->points.begin ();
        while (table_hull_it != table_convex_hull->points.end ())
        {
          dist_to_line = lineToPointDistance<PointT> (line_point, *table_hull_it, *p_it);
          if (dist_to_line < min_dist)
          {
            min_dist = dist_to_line;
          }
          line_point = *table_hull_it++;
        }
        if (min_dist > max_min_dist)
        {
          max_min_dist = min_dist;
        }
        if (max_min_dist > threshold)
        {
          return true;
        }
        p_it++;
      }
      // last if could be removed...
      if (max_min_dist > threshold)
      {
        return true;
      }

      return false;
    }

  template <typename PointT>
    void mergeRemainingHulls (const std::vector<boost::shared_ptr<::pcl::PointCloud<PointT> > > &all_hulls,
       const std::vector<std::vector<Eigen::Vector2i> > &remaining_hull_coords,
       std::vector<boost::shared_ptr<::pcl::PointCloud<PointT> > > &fused_hull_clouds,
       std::vector<std::vector<Eigen::Vector2i> > &hull_cluster_coords,
       float max_fusion_dist)
    {
      // clear output argument
      fused_hull_clouds.clear ();
      hull_cluster_coords.clear ();

      // compute bounding boxes for all borders
      std::vector<Eigen::Vector4f> min_bboxes, max_bboxes;
      min_bboxes.reserve (all_hulls.size ());
      max_bboxes.reserve (all_hulls.size ());
      Eigen::Vector4f min_bbox, max_bbox;
      size_t total_hull_points = 0;
      for (size_t i = 0; i < all_hulls.size (); ++i)
      {
        ::pcl::getMinMax3D<PointT> (*all_hulls[i], min_bbox, max_bbox);
        min_bboxes.push_back (min_bbox);
        max_bboxes.push_back (max_bbox);
        total_hull_points += all_hulls[i]->points.size ();
      }

      float max_fusion_dist2 = max_fusion_dist * max_fusion_dist;
      bool detected_set;
      std::pair<std::set<size_t>::iterator, bool> insert_res;
      std::vector<int> cluster_correspondence (all_hulls.size (), -1);

      size_t nr_clusters = 0;
      int next_cluster_nr = 0;
      int c_index;
      for (size_t i = 0; i < all_hulls.size (); ++i)
      {
        // retrieve the current hull
        boost::shared_ptr<const ::pcl::PointCloud<PointT> > curr_hull = all_hulls[i];
        // check if current hull isn't part of any cluster yet
        if (cluster_correspondence[i] == -1)
        {
          cluster_correspondence[i] = next_cluster_nr++;
          nr_clusters++;
        }
        c_index = cluster_correspondence[i];
        for (size_t j = i + 1; j < all_hulls.size (); ++j)
        {
          // check if compared hull isn't already in the same cluster
          if (c_index != cluster_correspondence[j])
          {
            // retrieve the hull to compare with
            boost::shared_ptr<const ::pcl::PointCloud<PointT> > compare_hull = all_hulls[j];

            // first check if bounding boxes are close enough (necessary criteria)
            float bbox_dist = holeBorderLowerBoundDist2 (min_bboxes[i], max_bboxes[i],
                min_bboxes[j], max_bboxes[j]);
            if (max_fusion_dist2 > bbox_dist)
            {
              bool fuse = false;
              // check if the hulls intersect
              if (bbox_dist <= 2.0f * std::numeric_limits<float>::epsilon ())
              {
                // TODO: hulls could once all be converted to speed up doConvexHulls2DIntersect ()
                fuse = doConvexHulls2DIntersect<PointT> (curr_hull, compare_hull);
              }
              if (!fuse)  // if hulls don't overlap, check if they are close enough
              {
                fuse =  convexHullDistBelowThreshold<PointT> (curr_hull, compare_hull, max_fusion_dist);
              }
              if (fuse) // intersect or close enough
              {
                // is the hull that is to be fused already part of another cluster?
                if (cluster_correspondence[j] != -1)
                {
                  // add the current hull and all other hulls from the same cluster to the cluster of the compared hull
                  int fusion_index = cluster_correspondence[j];
                  for (size_t k = 0; k < cluster_correspondence.size (); ++k)
                  {
                    if (cluster_correspondence[k] == c_index)
                    {
                      cluster_correspondence[k] = fusion_index;
                    }
                  }
                  c_index = fusion_index;
                  nr_clusters--;
                }
                else
                {
                  // add compare hull to cluster of current hull
                  cluster_correspondence[j] = c_index;
                }
              } // else: hulls are farther apart than threshold
            } //  else: bounding boxes are farther apart than threshold
          } // hulls are already in the same cluster
        } // checked current hull against all other hulls for fusion - investigate next hull
      }

      // determine how many clusters exist
      std::vector<int>::const_iterator c_index_it = cluster_correspondence.begin ();
      std::set<int> cluster_nr_set;
      while (c_index_it != cluster_correspondence.end ())
      {
        cluster_nr_set.insert (*c_index_it++);
      }
      fused_hull_clouds.reserve (cluster_nr_set.size ());
      hull_cluster_coords.reserve (cluster_nr_set.size ());
      std::set<int>::const_iterator set_it = cluster_nr_set.begin ();
      // aggregate points and 2D coords of convex hulls in the same cluster
      while (set_it != cluster_nr_set.end ())
      {
        auto current_hull_cluster = boost::make_shared<::pcl::PointCloud<PointT> > ();
        std::vector<Eigen::Vector2i> current_hull_cluster_coords;
        current_hull_cluster->points.reserve (total_hull_points);
        current_hull_cluster_coords.reserve (total_hull_points);
        size_t last_cluster_member_hull;
        for (size_t i = 0; i < cluster_correspondence.size (); ++i)
        {
          if (cluster_correspondence[i] == *set_it)
          {
            current_hull_cluster->points.insert (current_hull_cluster->points.end (),
                all_hulls[i]->points.begin (), all_hulls[i]->points.end ());
            current_hull_cluster_coords.insert (current_hull_cluster_coords.end (),
                remaining_hull_coords[i].begin (), remaining_hull_coords[i].end ());
            last_cluster_member_hull = i;
          }
        }
        // add aggregated points / 2D coords to output arguments
        current_hull_cluster->header = all_hulls[last_cluster_member_hull]->header;
        fused_hull_clouds.push_back (current_hull_cluster);
        hull_cluster_coords.push_back (current_hull_cluster_coords);
        set_it++;
      }
    }

  static void declare_params (tendrils& params)
  {
    params.declare<size_t> ("min_hole_size", "Minimal numbers of connected pixels in the depth image to form a hole", 15);
    params.declare<float> ("inside_out_factor", "Determines if a nan-region is outside of table hull (#outside > #points_inside * inside_out_factor)", 2.0f);
    params.declare<float> ("plane_dist_threshold", "Distance threshold for plane classification", .02f);
    params.declare<float> ("min_distance_to_convex_hull", "Minimal distance to convex hull for overlapping holes", .05f);
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
    min_distance_to_convex_hull_ = params["min_distance_to_convex_hull"];
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
      Eigen::Vector3f plane_normal = Eigen::Vector3f ((*model_)->values[0],
          (*model_)->values[1], (*model_)->values[2]);

      Eigen::Vector2i table_min, table_max;
      std::vector<Eigen::Vector2i> hull_2Dcoords;
      std::vector<std::vector<Eigen::Vector2i> > all_hole_2Dcoords;
      std::vector<std::vector<Eigen::Vector2i> > all_border_2Dcoords;
      getBoundingBox2DConvexHull (input, **hull_indices_, table_min, table_max, hull_2Dcoords);

      // retrieve the 3D coordinates of the convex hull of the tabletop
      auto table_convex_hull = boost::make_shared<::pcl::PointCloud<PointT> > ();
      table_convex_hull->points.reserve ((*hull_indices_)->indices.size ());
      std::vector<int>::const_iterator hull_index_it = (*hull_indices_)->indices.begin ();
      while (hull_index_it != (*hull_indices_)->indices.end ())
      {
        table_convex_hull->points.push_back (input->points[*hull_index_it++]);
      }
      table_convex_hull->width = table_convex_hull->points.size ();
      table_convex_hull->height = 1;

      std::vector<std::vector<bool> > visited (input->width, std::vector<bool> (input->height, false));

      // iterative region growing
      iterativeNANGrowing (input, hull_2Dcoords, table_min, table_max,
          all_hole_2Dcoords, all_border_2Dcoords);
      std::cout << "finished iterativeNANGrowing () call" << std::endl;

      /*
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
      */

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

      std::vector<boost::shared_ptr<::pcl::PointCloud<PointT> > > remaining_hulls;
      std::vector<std::vector<Eigen::Vector2i> > remaining_hull_coords;
      remaining_hulls.reserve (inside_borders.size () + overlap_borders.size ());
      remaining_hull_coords.reserve (inside_borders.size () + overlap_borders.size ());

      // create array to store all non nan-points that are enclosed by the hole
      // and should be removed before clustering
      ::pcl::PointIndices::Ptr remove_indices (new ::pcl::PointIndices);
      std::vector<Eigen::Vector2i>::const_iterator coord_it;

      Eigen::Vector4f plane_coefficients ((*model_)->values[0], (*model_)->values[1],
          (*model_)->values[2], (*model_)->values[3]);

      // create representations for the holes completely inside convex hull of table top
      auto holes_msg= boost::make_shared<transparent_object_reconstruction::Holes>();
      holes_msg->convex_hulls.reserve (inside_holes.size ());
      for (size_t i = 0; i < inside_borders.size (); ++i)
      {
        auto border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        erodeBorder (input, inside_borders[i], border_cloud, 2);

        double dist_sum = .0f;
        auto border_it = border_cloud->points.begin ();

        // check if hole border aligns with tabletop (or hole was caused by noise of tabletop obj)
        while (border_it != border_cloud->points.end ())
        {
          dist_sum += ::pcl::pointToPlaneDistance (*border_it++, plane_coefficients);
        }

        double avg_dist = dist_sum / static_cast<double> (border_cloud->points.size ());
        if (avg_dist > *plane_dist_threshold_)
        {
          // skip hole! should something more be done?
          // TODO: remove measurement points inside hole anyway?
          continue;
        }

        // project border into plane and retrieve convex hull
        auto conv_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        ::pcl::PointIndices conv_border_indices;
        projectBorderAndCreateHull (border_cloud, conv_border_cloud, conv_border_indices);

        std::vector<Eigen::Vector2i> convex_hull_polygon;
        convex_hull_polygon.reserve (conv_border_indices.indices.size ());
        std::vector<int>::const_iterator hull_it = conv_border_indices.indices.begin ();
        while (hull_it != conv_border_indices.indices.end ())
        {
          convex_hull_polygon.push_back (inside_borders[i][*hull_it++]);
        }
        // store indices of points that are inside the convex hull - these will be removed later
        bool touches_border;
        Eigen::Vector2i min_bbox, max_bbox;
        // check if hole touches border
        get2DHullBBox (input, convex_hull_polygon, min_bbox, max_bbox, touches_border);
        if (!touches_border)
        {
          remaining_hulls.push_back (conv_border_cloud);
          remaining_hull_coords.push_back (convex_hull_polygon);

        }
        // else discard the hole
      }

      // work on the holes that are partially inside the convex hull
      for (size_t i = 0; i < overlap_borders.size (); ++i)
      {
        // first retrieve the 3D points from the extracted border, project into plane and distinguish which are inside the table convex hull
        auto extracted_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        auto inside_hull_border = boost::make_shared<::pcl::PointCloud<PointT> > ();
        extracted_border_cloud->points.reserve (overlap_borders[i].size ());
        inside_hull_border->points.reserve (overlap_borders[i].size ());
        PointT projection;
        double dist_sum = 0.0f;
        std::vector<Eigen::Vector2i>::const_iterator coord_it = overlap_borders[i].begin ();
        while (coord_it != overlap_borders[i].end ())
        {
          if (projectPointOnPlane<PointT> (input->at ((*coord_it)[0], (*coord_it)[1]), projection, plane_coefficients))
          {
            extracted_border_cloud->points.push_back (projection);
            if (pointInPolygon2D (hull_2Dcoords, *coord_it))
            {
              dist_sum += ::pcl::pointToPlaneDistance (input->at ((*coord_it)[0], (*coord_it)[1]), plane_coefficients);
              inside_hull_border->points.push_back (projection);
            }
          }
          // otherwise discard the point
          coord_it++;
        }
        // set dimension
        extracted_border_cloud->width = extracted_border_cloud->points.size ();
        extracted_border_cloud->height = 1;
        inside_hull_border->width = inside_hull_border->points.size ();
        inside_hull_border->height = 1;

        double avg_dist = dist_sum / static_cast<double> (inside_hull_border->points.size ());
        if (avg_dist > *plane_dist_threshold_ * 3.0f) // TODO: perhaps remove the points with the largest 3 distances instead?
        {
          //skip hole! should something more be done?
          continue;
        }

        unsigned int inside_points = 0;
        // create a marker to check which of the border points are inside the convex hull
        std::vector<bool> point_inside (overlap_borders[i].size (), false);

        coord_it = overlap_borders[i].begin ();
        if (extracted_border_cloud->points.size () > 0 &&
            minDistAboveThreshold (table_convex_hull, inside_hull_border, *min_distance_to_convex_hull_))
        {
          // erode border cloud
          auto border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
          erodeBorder (input, overlap_borders[i], border_cloud, 2);

          // project border into plane and retrieve convex hull
          ::pcl::PointIndices conv_border_indices;
          auto conv_proj_border_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
          projectBorderAndCreateHull (border_cloud, conv_proj_border_cloud, conv_border_indices);

          std::vector<Eigen::Vector2i> convex_hull_polygon;
          convex_hull_polygon.reserve (conv_border_indices.indices.size ());
          std::vector<int>::const_iterator hull_it = conv_border_indices.indices.begin ();
          while (hull_it != conv_border_indices.indices.end ())
          {
            convex_hull_polygon.push_back (overlap_borders[i][*hull_it++]);
          }
          // store indices of points that are inside the convex hull - these will be removed later
          bool touches_border;
          Eigen::Vector2i min_bbox, max_bbox;
          // check if hole touches border
          get2DHullBBox (input, convex_hull_polygon, min_bbox, max_bbox, touches_border);
          if (!touches_border)
          {
            remaining_hulls.push_back (conv_proj_border_cloud);
            remaining_hull_coords.push_back (convex_hull_polygon);
          }
          // else discard the hole
        }
      }

      // check if some the remaining holes should be merged depending on their distance
      std::vector<boost::shared_ptr<::pcl::PointCloud<PointT> > > fused_hull_clouds;
      std::vector<std::vector<Eigen::Vector2i> > hull_cluster_coords;
      // fuse convex hulls that are close enough to each other
      mergeRemainingHulls<PointT> (remaining_hulls, remaining_hull_coords,
         fused_hull_clouds, hull_cluster_coords, 0.05f);

      ::pcl::PointCloud<::pcl::PointXYZRGBL>::Ptr output_cloud (new ::pcl::PointCloud<::pcl::PointXYZRGBL>);
      ROS_DEBUG_STREAM_NAMED("HoleDetector", "mergeRemainingHulls finished, fused_hull_clouds.size (): " << fused_hull_clouds.size ());

      static size_t f_counter = 0;
      ::pcl::PCDWriter test_writer;
      for (size_t i = 0; i < fused_hull_clouds.size (); ++i)
      {
        // create a new cloud for the convex hull of a cloud cluster
        auto hull_cloud = boost::make_shared<::pcl::PointCloud<PointT> > ();
        hull_cloud->header = fused_hull_clouds[i]->header;

        // now do a convex hull computation
        ::pcl::PointIndices convex_hull_indices;
        ::pcl::ConvexHull<PointT> c_hull;
        c_hull.setInputCloud (fused_hull_clouds[i]);
        c_hull.setDimension (2);
        c_hull.reconstruct (*hull_cloud);
        c_hull.getHullPointIndices (convex_hull_indices);

        // retrieve the 2D coordinates for the resulting hull
        std::vector<int>::const_iterator c_index_it = convex_hull_indices.indices.begin ();
        std::vector<Eigen::Vector2i> current_hull_coords;
        current_hull_coords.reserve (convex_hull_indices.indices.size ());
        while (c_index_it != convex_hull_indices.indices.end ())
        {
          current_hull_coords.push_back (hull_cluster_coords[i][*c_index_it++]);
        }

        // check if there are measurement points inside the current hull that should be removed
        bool touches_border;
        addRemoveIndices (input, current_hull_coords, remove_indices, touches_border);

        // add to hole msgs
        hull_cloud->header = input->header;
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg (*hull_cloud, pc2);
        holes_msg->convex_hulls.push_back (pc2);
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

      // TODO: do we still need the filtered point cloud as output?
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
  ecto::spore<float> min_distance_to_convex_hull_;
  ecto::spore<::pcl::PointIndices::ConstPtr> hull_indices_;
  ecto::spore<::pcl::ModelCoefficients::ConstPtr> model_;
  ecto::spore<ecto::pcl::PointCloud> output_;
  ecto::spore<transparent_object_reconstruction::Holes::ConstPtr> holes_mgs_;
  ecto::spore<::pcl::PointIndices::ConstPtr> remove_indices_;
};

ECTO_CELL(hole_detection, ecto::pcl::PclCell<HoleDetector>,
    "HoleDetector", "Extract a new cloud given an existing cloud and a set of indices to extract.");

