#include <transparent_object_reconstruction/tools.h>

template <>
Eigen::Vector3f
convert<Eigen::Vector3f, ColorPoint> (const ColorPoint &p)
{
  return Eigen::Vector3f (p.x, p.y, p.z);
}

template <>
Eigen::Vector4f
convert<Eigen::Vector4f, ColorPoint> (const ColorPoint &p)
{
  return Eigen::Vector4f (p.x, p.y, p.z, 0.0f);
}

template <>
Eigen::Vector3f
convert<Eigen::Vector3f, LabelPoint> (const LabelPoint &p)
{
  return Eigen::Vector3f (p.x, p.y, p.z);
}

template <>
Eigen::Vector4f
convert<Eigen::Vector4f, LabelPoint> (const LabelPoint &p)
{
  return Eigen::Vector4f (p.x, p.y, p.z, 0.0f);
}

template <>
Eigen::Vector3d
convert<Eigen::Vector3d, LabelPoint> (const LabelPoint &p)
{
  return Eigen::Vector3d (p.x, p.y, p.z);
}

template <>
LabelPoint
convert<LabelPoint, Eigen::Vector3d> (const Eigen::Vector3d &v)
{
  LabelPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <>
ColorPoint
convert<ColorPoint, Eigen::Vector3f> (const Eigen::Vector3f &v)
{
  ColorPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <>
ColorPoint
convert<ColorPoint, Eigen::Vector4f> (const Eigen::Vector4f &v)
{
  ColorPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <>
LabelPoint
convert<LabelPoint, Eigen::Vector3f> (const Eigen::Vector3f &v)
{
  LabelPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <>
LabelPoint
convert<LabelPoint, Eigen::Vector4f> (const Eigen::Vector4f &v)
{
  LabelPoint p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <>
Eigen::Vector4f
convert<Eigen::Vector4f, Model> (const Model &model)
{
  return Eigen::Vector4f::Map (&model.values[0], 4);
}

template <>
Eigen::Vector4f
convert<Eigen::Vector4f, ModelPtr> (const ModelPtr &model)
{
  return Eigen::Vector4f::Map (&model->values[0], 4);
}

template <>
Model
convert<Model, Eigen::Vector4f> (const Eigen::Vector4f &v)
{
  Model m;
  for (size_t i = 0; i < 4; ++i)
  {
    m.values[i] = v[i];
  }
  return m;
}

template <>
ModelPtr
convert<ModelPtr, Eigen::Vector4f> (const Eigen::Vector4f &v)
{
  ModelPtr m (new Model);
  for (size_t i = 0; i < 4; ++i)
  {
    m->values[i] = v[i];
  }
  return m;
}
template <>
void
insert_coords<Eigen::Vector3f, ColorPoint> (const Eigen::Vector3f &source, ColorPoint &target)
{
  target.x = source[0];
  target.y = source[1];
  target.z = source[2];
}

template <>
void
insert_coords<Eigen::Vector4f, ColorPoint> (const Eigen::Vector4f &source, ColorPoint &target)
{
  target.x = source[0];
  target.y = source[1];
  target.z = source[2];
}

template <>
void
insert_coords<Eigen::Vector3f, LabelPoint> (const Eigen::Vector3f &source, LabelPoint &target)
{
  target.x = source[0];
  target.y = source[1];
  target.z = source[2];
}

template <>
void
insert_coords<Eigen::Vector4f, LabelPoint> (const Eigen::Vector4f &source, LabelPoint &target)
{
  target.x = source[0];
  target.y = source[1];
  target.z = source[2];
}

void
extractVoxelGridCellPoints (const CloudPtr &cloud,
    pcl::VoxelGrid<ColorPoint> &v_grid,
    size_t nr_filled_grid_cells,
    std::vector<Cloud::VectorType> &cell_points)
{
  size_t reserve_points = 2 * cloud->points.size () / nr_filled_grid_cells;

  // clear output variables and allocate enough space
  cell_points.clear ();
  cell_points.resize (nr_filled_grid_cells);
  std::vector<Cloud::VectorType>::iterator c = cell_points.begin ();
  while (c != cell_points.end ())
  {
    c->reserve (reserve_points);
    c++;
  }
  // iterate over cloud points and insert them into their corresponding
  // grid cell
  Cloud::VectorType::const_iterator p_it = cloud->points.begin ();
  int cell_index;
  size_t points_not_in_grid = 0;
  while (p_it != cloud->points.end ())
  {
    cell_index = v_grid.getCentroidIndex (*p_it);
    if (cell_index != -1)
    {
      cell_points[cell_index].push_back (*p_it);
    }
    else
    {
      points_not_in_grid++;
    }
    p_it++;
  }
  if (points_not_in_grid > 0)
  {
    std::cerr << "There were " << points_not_in_grid << " that were not"
      << " associated with any grid cell (altough they should have been)."
      << std::endl;
  }
}

void
filterGridCells (const std::vector<Cloud::VectorType> &cell_points,
    size_t min_points_in_cell, std::vector<size_t> &indices)
{
  indices.clear ();
  indices.reserve (cell_points.size ());

  for (size_t i = 0; i < cell_points.size (); ++i)
  {
    if (cell_points[i].size () >= min_points_in_cell)
    {
      indices.push_back (i);
    }
  }
}

bool
sortCloudBySizeDesc (const CloudPtr &a, const CloudPtr &b)
{
  return a->points.size () > b->points.size ();
}

bool
alignPlaneCoefficientsToOrigin (ModelPtr plane_coefficients)
{
  // generate origin and plane a 4D vectors
  Eigen::Vector4f origin (0.0f, 0.0f, 0.0f, 1.0f);
  Eigen::Vector4f plane (plane_coefficients->values[0],
      plane_coefficients->values[1],
      plane_coefficients->values[2],
      plane_coefficients->values[3]);
  // check if origin is below plane
  if (origin.dot (plane) < 0.0f)
  {
    // change direction of normal vector and adapt 4th plane coefficient
    for (size_t i = 0; i < 4; ++i)
    {
      plane_coefficients->values[i] *= -1.0f;
    }
    return true;
  }
  // else nothing has to be done
  return false;
}

void calcPlaneTransformation (Eigen::Vector3f plane_normal,
    const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  Eigen::Vector3f ortho_to_normal = Eigen::Vector3f::Unit (3,1); // (0, 1, 0)

  float x, y, z;
  double angle = pcl::getAngle3D (Eigen::Vector4f (plane_normal[0],
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
  pcl::getTransformationFromTwoUnitVectorsAndOrigin (ortho_to_normal,
      plane_normal, origin, transformation);
}

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

void
calcTransformedPlaneEquations (const ModelVector &plane_coeff_vector,
    const std::vector<Eigen::Affine3f> &transformation_vector,
    TransPlaneCoeffsVec &transformed_plane_equations)
{
  // clear contents of transformed_plane_equations
  transformed_plane_equations.clear ();
  // allocate enough memory to store all transformed equations
  transformed_plane_equations.resize (transformation_vector.size ());

  float x, y, z;
  for (size_t i = 0; i < transformation_vector.size (); ++i)
  {
    Eigen::Affine3f transformation = transformation_vector[i];
    // create the normal vector of the first plane
    for (size_t k = 0; k < plane_coeff_vector.size (); ++k)
    {
      // retrieve the plane normal
      Eigen::Vector3f normal (plane_coeff_vector[k]->values[0],
          plane_coeff_vector[k]->values[1],
          plane_coeff_vector[k]->values[2]);
      // create a point on the plane
      // normal[2] is not 0
      if (fabs (normal[2]) > std::numeric_limits<float>::epsilon ())
      {
        x = y = 1.0f;
        z = -plane_coeff_vector[k]->values[3] - normal[0] - normal[1];
        z /= normal[2];
      }
      // normal[2] is 0 and normal[1] is not 0
      else if (fabs (normal[1]) > std::numeric_limits<float>::epsilon ())
      {
        x = z = 1.0f;
        y = -plane_coeff_vector[k]->values[3] - normal[0];
        y /= normal[1];
      }
      // normal[2] and normal[1] are both 0
      else
      {
        y = z = 1.0f;
        x = -plane_coeff_vector[k]->values[3] / normal[0];
      }
      Eigen::Vector3f point_on_plane (x, y, z);

      // check for correct calculation of point_on_plane
      float dot_prod;
      Eigen::Vector4f test_point (point_on_plane[0], point_on_plane[1], point_on_plane[2], 1.0f);
      Eigen::Vector4f tmp_plane (normal[0], normal[1], normal[2], plane_coeff_vector[k]->values[3]);
      dot_prod = test_point.dot (tmp_plane);
      assert (fabs (dot_prod) < 2 * std::numeric_limits<float>::epsilon ());

      Eigen::Vector3f rotated_normal = transformation.linear () * normal;
      rotated_normal.normalize ();
      Eigen::Vector3f trans_point_on_plane = transformation * point_on_plane;
      Eigen::Vector4f trans_plane_equation (rotated_normal[0],
          rotated_normal[1],
          rotated_normal[2],
          -rotated_normal.dot (trans_point_on_plane));
      transformed_plane_equations[i].push_back (trans_plane_equation);

      // check for correct transformation of plane
      test_point = Eigen::Vector4f (trans_point_on_plane[0],
          trans_point_on_plane[1],trans_point_on_plane[2], 1.0f);
      dot_prod = test_point.dot (trans_plane_equation);
      assert (fabs (dot_prod) < 2 * std::numeric_limits<float>::epsilon ());
    }
  }
}

// TODO: introduce second criteria (possibly distance based), since
// currently some valid points might be cropped (for example the floor
// if there is a table present (floor should be behind table plane if
// camera if above table).
void
cropPlaneByPlanes (CloudPtr cropped_plane, const CloudVector &plane_vector,
    const ModelVector &plane_coeff_vector, size_t plane_index)
{
  // reserve enough room in the cropped plane
  cropped_plane->points.clear ();
  cropped_plane->points.reserve (plane_vector[plane_index]->points.size ());
  ColorPoint p;
  bool behind_any_plane;
  size_t nr_points = 0;
  for (size_t i = 0; i < plane_vector[plane_index]->points.size (); ++i)
  {
    behind_any_plane = false;

    p = plane_vector[plane_index]->points[i];
    Eigen::Vector4f p_vec (p.x, p.y, p.z, 1.0);

    // check for all planes
    for (size_t j = 0; j < plane_vector.size (); ++j)
    {
      // don't crop with the current plane
      if (j == plane_index)
        continue;
      else
      {
        Eigen::Vector4f crop_plane_coeff (plane_coeff_vector[j]->values[0],
            plane_coeff_vector[j]->values[1],
            plane_coeff_vector[j]->values[2],
            plane_coeff_vector[j]->values[3]);
        // check if current point is behind the current plane
        if (p_vec.dot (crop_plane_coeff) < 0.0f)
        {
          behind_any_plane = true;
          break;
        }
      }
    }
    if (!behind_any_plane)
    {
      nr_points++;
      cropped_plane->points.push_back (p);
    }
  }
  // resize the vector to used size
  cropped_plane->points.resize (nr_points);
  // adapt height and width of pointcloud
  cropped_plane->height = 1;
  cropped_plane->width = nr_points;
}

void
cropPlaneByPlanes (CloudVector &plane_vector,
    const ModelVector &plane_coeff_vector, size_t plane_index)
{
  Cloud tmp_cloud;
  tmp_cloud.points.reserve (plane_vector[plane_index]->points.size ());
  pcl::PointCloud<ColorPoint>::iterator p = plane_vector[plane_index]->begin ();
  bool above_plane;
  size_t nr_points = 0;
  while (p != plane_vector[plane_index]->end ())
  {
    above_plane = true;
    Eigen::Vector4f point_vec (p->x, p->y, p->z, 1.0f);
    // check for all other planes
    for (size_t i = 0; i < plane_vector.size (); ++i)
    {
      if (i == plane_index)
        continue;
      else
      {
        Eigen::Vector4f plane_vec (plane_coeff_vector[i]->values[0],
            plane_coeff_vector[i]->values[1],
            plane_coeff_vector[i]->values[2],
            plane_coeff_vector[i]->values[3]);
        // check if the current point is behind the current plane
        if (point_vec.dot (plane_vec) < 0.0)
        {
          above_plane = false;
          break;
        }
      }
    }
    if (above_plane)
    {
      nr_points++;
      tmp_cloud.points.push_back (*p);
    }
    p++;
  }
  // resize the vector to used size
  tmp_cloud.points.resize (nr_points);
  // adapt the height and width
  plane_vector[plane_index]->height = 1;
  plane_vector[plane_index]->width = nr_points;
  // swap the point information
  plane_vector[plane_index]->points.swap (tmp_cloud.points);
}

void
cropPointCloudByAllPlanes (CloudPtr cloud, const ModelVector &plane_coeff_vector)
{
  Cloud tmp_cloud;
  tmp_cloud.points.reserve (cloud->points.size ());

  size_t nr_points = 0;
  bool above_plane;
  ColorPoint point;
  Cloud::iterator p = cloud->begin ();
  // check for every point in the pointcloud
  while (p != cloud->end ())
  {
    above_plane = true;
    Eigen::Vector4f point_vec (p->x, p->y, p->z, 1.0f);
    // check for every plane
    for (size_t i = 0; i < plane_coeff_vector.size (); ++i)
    {
      Eigen::Vector4f plane_vec (plane_coeff_vector[i]->values[0],
          plane_coeff_vector[i]->values[1],
          plane_coeff_vector[i]->values[2],
          plane_coeff_vector[i]->values[3]);
      // check if the current point is behind the current plane
      if (point_vec.dot (plane_vec) < 0.0)
      {
        above_plane = false;
        break;
      }
    }
    if (above_plane)
    {
      nr_points++;
      tmp_cloud.points.push_back (*p);
    }
    p++;
  }
  // resize the vector to used size
  tmp_cloud.points.resize (nr_points);
  // adapt height and width of cloud
  cloud->height = 1;
  cloud->width = nr_points;
  // copy the point information
  cloud->points.swap (tmp_cloud.points);
}

void
cropPointCloudByPlanes (CloudPtr cloud,
    const ModelVector &plane_coeff_vector,
    size_t not_used_plane_index)
{
  Cloud tmp_cloud;
  tmp_cloud.points.reserve (cloud->points.size ());

  size_t nr_points = 0;
  bool above_plane;
  ColorPoint point;
  Cloud::iterator p = cloud->begin ();
  // check for every point in the pointcloud
  while (p != cloud->end ())
  {
    above_plane = true;
    Eigen::Vector4f point_vec (p->x, p->y, p->z, 1.0f);
    for (size_t i = 0; i < plane_coeff_vector.size (); ++i)
    {
      if (i != not_used_plane_index)
      {
        Eigen::Vector4f plane_vec (plane_coeff_vector[i]->values[0],
            plane_coeff_vector[i]->values[1],
            plane_coeff_vector[i]->values[2],
            plane_coeff_vector[i]->values[3]);
        // check if the current point is behind the current plane
        if (point_vec.dot (plane_vec) < 0.0)
        {
          above_plane = false;
          break;
        }
      }
    }
    if (above_plane)
    {
      nr_points++;
      tmp_cloud.points.push_back (*p);
    }
    p++;
  }
  // resize the vector to used size
  tmp_cloud.points.resize (nr_points);
  // adapt height and width of cloud
  cloud->height = 1;
  cloud->width = nr_points;
  // copy the point information
  cloud->points.swap (tmp_cloud.points);
}

void
cropPointCloudBySpecifiedPlanes (CloudPtr cloud,
    const ModelVector &plane_coeff_vector,
    const std::vector<size_t> &crop_indices)
{
  Cloud tmp_cloud;
  tmp_cloud.points.reserve (cloud->points.size ());

  // set up the ModelVector for the acutally used planes:
  ModelVector crop_planes;
  crop_planes.reserve (plane_coeff_vector.size ());
  size_t nr_planes = 0;
  for (size_t i = 0; i < crop_indices.size (); ++i)
  {
    // check if the index is inside the limits
    if (crop_indices[i] < plane_coeff_vector.size ())
    {
      ModelPtr tmp_model (new Model (*(plane_coeff_vector[crop_indices[i]])));
      crop_planes.push_back (tmp_model);
      nr_planes++;
    }
  }
  crop_planes.resize (nr_planes);

  size_t nr_points = 0;
  bool above_plane;
  ColorPoint point;
  Cloud::iterator p = cloud->begin ();
  // check for every point in the pointcloud
  while (p != cloud->end ())
  {
    above_plane = true;
    Eigen::Vector4f point_vec (p->x, p->y, p->z, 1.0f);
    for (size_t i = 0; i < crop_planes.size (); ++i)
    {
      Eigen::Vector4f plane_vec (crop_planes[i]->values[0],
          crop_planes[i]->values[1],
          crop_planes[i]->values[2],
          crop_planes[i]->values[3]);
      // check if the current point is behind the current plane
      if (point_vec.dot (plane_vec) < 0.0)
      {
        above_plane = false;
        break;
      }
    }
    if (above_plane)
    {
      nr_points++;
      tmp_cloud.points.push_back (*p);
    }
    p++;
  }
  // resize the vector to used size
  tmp_cloud.points.resize (nr_points);
  // adapt height and width of cloud
  cloud->height = 1;
  cloud->width = nr_points;
  // copy the point information
  cloud->points.swap (tmp_cloud.points);

  crop_planes.clear ();
}

bool
pointInsideConvexPolygon (const std::vector<Eigen::Vector3f> &polygon, const Eigen::Vector3f &query_point)
{
  double angle_sum = 0;
  float sqrd_dist1, sqrd_dist2;
  Eigen::Vector3f p1, p2;
  std::vector<Eigen::Vector3f>::const_iterator p_it;
  p_it = --polygon.end ();
  p2 = (*p_it) - query_point;
  sqrd_dist2 = p2.squaredNorm ();
  if (sqrd_dist2 < MAX_SQRD_CORR_DIST)  // query point is (almost) identical with p2
  {
    return true;
  }
  p_it = polygon.begin ();

  while (p_it != polygon.end ())
  {
    p1 = (*p_it) - query_point;
    sqrd_dist1 = p1.squaredNorm ();
    // is query point (almost) identical with p1?
    if (sqrd_dist1 < MAX_SQRD_CORR_DIST)
    {
      return true;
    }
    angle_sum += acos (p1.dot (p2) / sqrt (sqrd_dist1 * sqrd_dist2));
    p2 = p1;
    sqrd_dist2 = sqrd_dist1;
    p_it++;
  }

  if ( fabs (angle_sum - 2 * M_PI) < LINE_PLANE_ANGLE_EPS)
    return true;
  return false;
}

void
cropClusterHullByPlaneHull (Cloud::ConstPtr plane_convex_hull,
    Cloud::ConstPtr proj_cluster_hull, Cloud &filtered_cluster_hull)
{
  filtered_cluster_hull.points.clear ();
  size_t nr_inliers = 0;
  filtered_cluster_hull.points.reserve (proj_cluster_hull->points.size ());
  ColorPoint query_point;
  for (size_t i = 0; i < proj_cluster_hull->points.size (); ++i)
  {
    query_point = proj_cluster_hull->points[i];
    if (pointInsideConvexPolygon (plane_convex_hull, query_point))
    {
      nr_inliers++;
      filtered_cluster_hull.points.push_back (query_point);
    }
  }
  filtered_cluster_hull.height = 1;
  filtered_cluster_hull.width = nr_inliers;
  filtered_cluster_hull.resize (nr_inliers);
}

bool 
lineWithPlaneIntersection (const Eigen::Vector3f point_a,
                           const Eigen::Vector3f point_b,
                           const Eigen::Vector4f &plane,
                           Eigen::Vector4f &point, double angle_eps)
{
  // first: construct the line from point_a and point_b
  Eigen::ParametrizedLine<float, 3> param_line = Eigen::ParametrizedLine<float, 3>::Through (point_a, point_b);

  // extract the plane normal from model coefficients 
  Eigen::Vector3f plane_normal = Eigen::Vector3f::Map (&plane[0], 3);

  // check if line and plane are parallel
  Eigen::Vector3f line_dir3f = param_line.direction ();
  Eigen::Vector4f line_dir4f (line_dir3f[0], line_dir3f[1], line_dir3f[2], 0.0f); 
  Eigen::Vector4f plane_normal4f (plane_normal[0], plane_normal[1], plane_normal[2], 0.0f);

  double angle = fabs (pcl::getAngle3D (line_dir4f, plane_normal4f));
  // is line parallel to plane (i.e. perpendicular to normal vector)?
  if (fabs (angle - M_PI / 2.0) < angle_eps)
  {
    point = Eigen::Vector4f::Zero ();
    return false;
  }

  // create the hyperplane
  Eigen::Hyperplane<float, 3> hyper_plane (plane_normal, plane[3]);

  // calculate the intersection
  float distance = param_line.intersection (hyper_plane);
  Eigen::Vector3f intersection = distance * param_line.direction () + param_line.origin ();
  point = Eigen::Vector4f (intersection[0], intersection[1], intersection[2], 0.0f); 
  return true;
}

void
refinePlanes (CloudVector &planes, ModelVector &plane_coefficients)
{
  // reserve enough memory for temporary storage
  unsigned int total_nr_points = 0;
  for (size_t i = 0; i < planes.size (); ++i)
  {
    total_nr_points += planes[i]->points.size ();
  }
  CloudVector tmp;
  tmp.reserve (planes.size ());
  for (size_t i = 0; i < planes.size (); ++i)
  {
    CloudPtr tmp_cloud (new Cloud);
    tmp_cloud->points.reserve (total_nr_points);
    tmp.push_back (tmp_cloud);
  }

  // create a Eigen::Hyperplane for each plane for distance computation
  std::vector<Eigen::Hyperplane<float, 3> > hyper_planes;
  Eigen::Vector3f normal;
  hyper_planes.resize (plane_coefficients.size ());
  for (size_t i = 0; i < plane_coefficients.size (); ++i)
  {
    normal = Eigen::Vector3f (plane_coefficients[i]->values[0],
        plane_coefficients[i]->values[1],
        plane_coefficients[i]->values[2]);
    hyper_planes[i] = Eigen::Hyperplane<float, 3> (normal,
        plane_coefficients[i]->values[3]);
  }

  float min_dist;
  size_t best_index;
  float dist;
  ColorPoint current_point;
  Eigen::Vector3f p;

  // for all planes
  for (size_t i = 0; i < planes.size (); ++i)
  {
    // associate current inlier with closest plane
    for (size_t j = 0; j < planes[i]->points.size (); ++j)
    {
      min_dist = PLANE_INLIER_THRESHOLD;
      best_index = i;
      current_point = planes[i]->points[j];
      p = Eigen::Vector3f (current_point.x, current_point.y,
          current_point.z);

      for (size_t k = 0; k < hyper_planes.size (); ++k)
      {
        dist = fabs (hyper_planes[k].signedDistance (p));
        if (dist < min_dist)
        {
          min_dist = dist;
          best_index = k;
        }
      }
      tmp[best_index]->points.push_back (current_point);
    }
  }

  // change contents of in/out point cloud vector with the refined clouds
  for (size_t i = 0; i < tmp.size (); ++i)
  {
    tmp[i]->height = 1;
    tmp[i]->width = tmp[i]->points.size ();
    tmp[i]->points.resize (tmp[i]->width);
    planes[i] = tmp[i];
  }
}

void 
projectCloudOnPlane (Cloud::ConstPtr input, CloudPtr projected_cloud, ModelPtr plane)
{
  // clear the points of projected cloud and reserve enough memory
  projected_cloud->points.clear ();
  projected_cloud->points.reserve (input->points.size ());
  ColorPoint intersection;
  Eigen::Vector4f plane_coeff = Eigen::Vector4f (plane->values[0],
      plane->values[1], plane->values[2], plane->values[3]);

  // iterate over input cloud
  for (size_t i = 0; i < input->points.size (); ++i)
  {
    if (projectPointOnPlane (input->points[i], intersection, plane_coeff))
    {
      projected_cloud->points.push_back (intersection);
    }
  }

  // adapt width and height of the projected cloud
  projected_cloud->height = 1;
  projected_cloud->width = projected_cloud->points.size ();
}

void
createEuclideanClusters (Cloud::ConstPtr input, CloudVector &output,
   unsigned int max_cluster_size = MAX_CLUSTER_SIZE,
   unsigned int min_cluster_size = MIN_CLUSTER_SIZE,
   double cluster_tolerance = CLUSTER_TOLERANCE)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
  tree->setInputCloud (input);
  
  // set up the parameters for the euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<ColorPoint> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input);
  ec.extract (cluster_indices);

  // clear the output vector
  output.clear ();
  // reserve space for each cluster
  output.reserve (cluster_indices.size ());

  // loop to store every cluster as a separate point cloud in output vector
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
      it != cluster_indices.end (); ++it)
  {
    pcl::ExtractIndices<ColorPoint> extract_object_indices;
    extract_object_indices.setInputCloud (input);
    extract_object_indices.setIndices (pcl::PointIndices::Ptr (new pcl::PointIndices(*it)));
    CloudPtr tmp (new Cloud);
    extract_object_indices.filter (*tmp);
    output.push_back (tmp);
    //std::cout << "PointCloud representing the Cluster: " << tmp->points.size () << " data points." << std::endl;
  }
}

void
projectCloudToPlane (Cloud::ConstPtr input, CloudPtr projected_cloud,
    ModelPtr plane_coefficients)
{
  pcl::ProjectInliers<ColorPoint> projection;
  projection.setModelType (pcl::SACMODEL_PLANE);
  projection.setInputCloud (input);
  projection.setModelCoefficients (plane_coefficients);
  projection.filter (*projected_cloud);
}

void
createHullLinesVec (CloudVector &hull_vector, CloudVector &hull_lines_vec,
    float step_length = 0.005f)
{
  // remove any content in the output and resize appropriately
  hull_lines_vec.clear ();
  hull_lines_vec.reserve (hull_vector.size ());

  CloudVector::iterator current_hull = hull_vector.begin ();
  while (current_hull != hull_vector.end ())
  {
    CloudPtr hull_lines (new Cloud);
    createHullLines<ColorPoint> (*current_hull, hull_lines, step_length);
    hull_lines_vec.push_back (hull_lines);
    current_hull++;
  }
}

void
createVoxelGrids (CloudVector &cloud_vector,
    std::vector<pcl::VoxelGrid<ColorPoint> > &v_grid_vector,
    CloudVector &grid_clouds,
    float x_grid_size = 0.01f,
    float y_grid_size = 0.01f,
    float z_grid_size = std::numeric_limits<float>::infinity ())
{
  // clean grid vector and grid cloud and allocate memory
  v_grid_vector.clear ();
  grid_clouds.clear ();
  v_grid_vector.resize (cloud_vector.size ());
  grid_clouds.resize (cloud_vector.size ());

  for (size_t i = 0; i < cloud_vector.size (); ++i)
  {
    CloudPtr grid_cloud (new Cloud);
    grid_clouds[i] = grid_cloud;
    v_grid_vector[i].setInputCloud (cloud_vector[i]);
    v_grid_vector[i].setLeafSize (x_grid_size, y_grid_size, z_grid_size);
    v_grid_vector[i].setSaveLeafLayout (true);
    v_grid_vector[i].filter (*grid_clouds[i]);
  }
}

size_t
plane (Cloud::ConstPtr input, CloudPtr &non_planar_cloud,
    CloudPtr &plane_cloud, ModelPtr plane_coefficients)
{

  //ModelPtr coefficients (new Model);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<ColorPoint> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // TODO: this could also be specified via (optional) command line param
  seg.setDistanceThreshold (PLANE_INLIER_THRESHOLD);

  seg.setInputCloud (input);
  seg.segment (*inliers, *plane_coefficients);

  // adapt (if necessary) plane coeffs so that origin is above plane
  if (alignPlaneCoefficientsToOrigin (plane_coefficients))
  {
    std::cout << "aligned plane coeffs" << std::endl;
  }

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return inliers->indices.size ();
  }
  
  pcl::ExtractIndices<ColorPoint> extract_object_indices;
  extract_object_indices.setNegative (true);
  extract_object_indices.setInputCloud (input);
  extract_object_indices.setIndices (inliers);
  CloudPtr tmp (new Cloud);
  extract_object_indices.filter (*tmp);

  non_planar_cloud = tmp;
  CloudPtr tmp2 (new Cloud);
  extract_object_indices.setNegative (false);
  extract_object_indices.filter (*tmp2);
  plane_cloud = tmp2;
  return inliers->indices.size ();
}

void
calcConvexHullsPerfectPlanes (CloudVector &perfect_planes_vector,
    CloudVector &conv_hulls)
{
  // first clean and allocate conv_hulls
  conv_hulls.clear ();
  conv_hulls.resize (perfect_planes_vector.size ());

  pcl::ConvexHull<ColorPoint> conv_hull;
  conv_hull.setDimension (2);
  // calculate the convex hull for every plane in plane_vector
  for (size_t i = 0; i < perfect_planes_vector.size (); ++i)
  {
    CloudPtr hull (new Cloud);
    conv_hull.setInputCloud (perfect_planes_vector[i]);
    conv_hull.reconstruct (*hull);
    conv_hulls[i] = hull;
  }
}

void
calcConvexHulls (CloudVector &plane_vector, CloudVector &conv_hulls,
    ModelVector &plane_coeff_vector)
{
  // create an allocate temporary vector for projected clouds
  CloudVector projected_clouds;
  projected_clouds.resize (plane_vector.size ());

  // calculate the convex hull for every plane in plane_vector
  for (size_t i = 0; i < plane_vector.size (); ++i)
  {
    CloudPtr projected_points (new Cloud);
    // project the inliers of the plane on the perfect plane
    projectCloudToPlane (plane_vector[i], projected_points,
        plane_coeff_vector[i]);
    projected_clouds[i] = projected_points;
  }
  calcConvexHullsPerfectPlanes (projected_clouds, conv_hulls);
}

size_t
removeNANs (Cloud::ConstPtr input, CloudPtr &cloud_without_nans)
{
  cloud_without_nans->points.clear ();
  cloud_without_nans->points.reserve (input->points.size ());
  Cloud::VectorType::const_iterator p_it;
  p_it = input->points.begin ();
  while (p_it != input->points.end ())
  {
    if (pcl::isFinite (*p_it))
    {
      cloud_without_nans->points.push_back (*p_it);
    }
    p_it++;
  }
  cloud_without_nans->width = cloud_without_nans->points.size ();
  cloud_without_nans->height = 1;
  return cloud_without_nans->points.size ();
}

void
createConcaveHulls (CloudVector &object_clusters,
    ModelVector &plane_coefficients,
    std::vector<Eigen::Affine3f> &plane_transformations,
    std::vector<CloudVector> &projected_clusters,
    std::vector<CloudVector> &concave_hulls,
    bool crop_by_other_planes = true)
{
  // clear any remaining content in the outputs
  projected_clusters.clear ();
  concave_hulls.clear ();

  // reserve memory for output
  projected_clusters.resize (plane_coefficients.size ());
  concave_hulls.resize (plane_coefficients.size ());

  for (size_t i = 0; i < plane_coefficients.size (); ++i)
  {
    projected_clusters[i].reserve (object_clusters.size ());
    concave_hulls[i].reserve (object_clusters.size ());
  }

  // create concave hull object
  pcl::ConcaveHull<ColorPoint> conc_hull;
  conc_hull.setAlpha (CONCAVE_HULL_ALPHA);

  // create the projected clusters
  for (size_t i = 0; i < plane_coefficients.size (); ++i)
  {
    for (size_t j = 0; j < object_clusters.size (); ++j)
    {
      CloudPtr projected_cloud (new Cloud);
      CloudPtr trans_proj_cloud (new Cloud);
      CloudPtr trans_conc_hull (new Cloud);
      CloudPtr concave_hull (new Cloud);
      projectCloudOnPlane (object_clusters[j], projected_cloud,
          plane_coefficients[i]);
      projected_clusters[i].push_back (projected_cloud);

      /*
       * transform the projected cluster into the x-y plane
       * note: this is needed for pcl::ConcaveHull to work properly atm
       */
      pcl::transformPointCloud (*projected_cloud, *trans_proj_cloud,
          plane_transformations[i]);

      // create the concave hull of the projected transformed cluster
      conc_hull.setInputCloud (trans_proj_cloud);
      conc_hull.reconstruct (*trans_conc_hull);

      // transform the concave hull back from x-y plane to original orientation
      pcl::transformPointCloud (*trans_conc_hull, *concave_hull,
          plane_transformations[i].inverse ());

      // crop points of concave hull by other planes
      if (crop_by_other_planes)
      {
        cropPointCloudByPlanes (concave_hull, plane_coefficients, i);
      }

      // add the concave hull to the vector
      concave_hulls[i].push_back (concave_hull);
    }
  }
}

void
cropConcaveHullsByPlaneHulls (const CloudVector &convex_hulls,
    std::vector<CloudVector> &concave_hulls)
{
  for (size_t i = 0; i < convex_hulls.size (); ++i)
  {
    for (size_t j = 0; j < concave_hulls[i].size (); ++j)
    {
      if (concave_hulls[i][j]->points.size () > 0)
      {
        CloudPtr cropped_hull (new Cloud);
        cropClusterHullByPlaneHull (convex_hulls[i], concave_hulls[i][j],
            *cropped_hull);
        if (concave_hulls[i][j]->points.size () >
            cropped_hull->points.size ())
        {
          concave_hulls[i][j] = cropped_hull;
        }
      }
    }
  }
}

void
initializeOutputVectors (const std::vector<CloudVector> &concave_cluster_hulls,
    CloudVector &correspondences,
    CloudVector &no_correspondences,
    CloudVector &scene_neighbors,
    CloudVector &complete_hulls)
{
  // clear any lingering content from the vectors
  correspondences.clear ();
  no_correspondences.clear ();
  scene_neighbors.clear ();
  complete_hulls.clear ();

  if (concave_cluster_hulls.empty ())
    return;

  correspondences.resize (concave_cluster_hulls[0].size ());
  no_correspondences.resize (concave_cluster_hulls[0].size ());
  scene_neighbors.resize (concave_cluster_hulls[0].size ());
  complete_hulls.resize (concave_cluster_hulls[0].size ());

  size_t max_points;
  for (size_t i = 0; i < correspondences.size (); ++i)
  {
    max_points = 0;
    CloudPtr corr_points (new Cloud);
    CloudPtr no_corr_points (new Cloud);
    CloudPtr scene_points (new Cloud);
    CloudPtr complete_hull (new Cloud);
    correspondences[i] = corr_points;
    no_correspondences[i] = no_corr_points;
    scene_neighbors[i] = scene_points;
    complete_hulls[i] = complete_hull;
    for (size_t j = 0; j < concave_cluster_hulls.size (); ++j)
    {
      max_points += concave_cluster_hulls[j][i]->points.size ();
    }
    correspondences[i]->points.reserve (max_points);
    no_correspondences[i]->points.reserve (max_points);
    scene_neighbors[i]->points.reserve (max_points);
    complete_hulls[i]->points.reserve (max_points);
  }
}

void
checkConcaveHullsNearestNeighbors (const CloudVector &planes,
    const std::vector<CloudVector> &concave_clusters,
    CloudVector &correspondences,
    CloudVector &no_correspondences,
    CloudVector &scene_neighbors,
    CloudVector &complete_hulls)
{
  initializeOutputVectors (concave_clusters, correspondences,
      no_correspondences, scene_neighbors, complete_hulls);

  // create KdTrees of the planes for the nearest neighbor search
  std::vector<pcl::KdTree<ColorPoint>::Ptr> plane_KdTrees;
  plane_KdTrees.reserve (planes.size ());
  for (size_t i = 0; i < planes.size (); ++i)
  {
    pcl::KdTree<ColorPoint>::Ptr tree (new pcl::KdTreeFLANN<ColorPoint>);
    tree->setInputCloud (planes[i]);
    plane_KdTrees.push_back (tree);
  }

  // variables needed for the nearest neigbor search
  unsigned int nr_neighbors;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_dist;
  k_indices.resize (1);
  k_sqr_dist.resize (1);
  ColorPoint query_point;

  for (size_t i = 0; i < planes.size (); ++i)
  {
    for (size_t j = 0; j < concave_clusters[i].size (); ++j)
    {
      for (size_t k = 0; k < concave_clusters[i][j]->points.size (); ++k)
      {
        query_point = concave_clusters[i][j]->points[k];
        complete_hulls[j]->points.push_back (query_point);
        // check the plane into which the cluster was projected
        nr_neighbors = plane_KdTrees[i]->radiusSearch (query_point,
            MAX_CORR_DIST, k_indices, k_sqr_dist, 1);
        if (nr_neighbors > 0)
        {
          correspondences[j]->points.push_back (query_point);
          scene_neighbors[j]->points.push_back
            (planes[i]->points[k_indices[0]]);
        }
        else
        {
          /*
           * check the other planes (there might be correspondences near
           * intersections between planes)
           */
          size_t plane_index;
          for (size_t l = 0; l < planes.size (); ++l)
          {
            if (l != i)
            {
              nr_neighbors = plane_KdTrees[l]->radiusSearch (query_point,
                  MAX_CORR_DIST, k_indices, k_sqr_dist, 1);
              if (nr_neighbors > 0)
              {
                plane_index = l;
                break;
              }
            }
          }
          if (nr_neighbors > 0)
          {
            correspondences[j]->points.push_back (query_point);
            scene_neighbors[j]->points.push_back
              (planes[plane_index]->points[k_indices[0]]);
          }
          else
          {
            no_correspondences[j]->points.push_back (query_point);
          }
        }
      }
    }
  }

  // set width and height of output clouds
  for (size_t i = 0; i < correspondences.size (); ++i)
  {
    correspondences[i]->height = 1;
    no_correspondences[i]->height = 1;
    scene_neighbors[i]->height = 1;
    complete_hulls[i]->height = 1;

    correspondences[i]->width = correspondences[i]->points.size ();
    no_correspondences[i]->width = no_correspondences[i]->points.size ();
    scene_neighbors[i]->width = scene_neighbors[i]->points.size ();
    complete_hulls[i]->width = complete_hulls[i]->points.size ();
  }
}

bool
intersectBoundingBoxes (const Eigen::Vector4f &box1_min,
    const Eigen::Vector4f &box1_max,
    const Eigen::Vector4f &box2_min,
    const Eigen::Vector4f &box2_max)
{
  float center_dist_a2, center_dist_b2;
  float boxes_dim_a, boxes_dim_b;
  float min_dist_threshold = 0.01f; // should be small enough for the precision we need here...

  // check if parallel to x-z-plane and project into x-z-plane
  if ((box1_max[1] - box1_min[1]) < min_dist_threshold &&
      (box2_max[1] - box2_min[1]) < min_dist_threshold)
  {
    // calculate the distance between the center points (times 2) in x and y
    center_dist_a2 = fabs (box1_min[0] + box1_max[0] -
        box2_min[0] - box2_max[0]);
    center_dist_b2 = fabs (box1_min[2] + box1_max[2] -
        box2_min[2] - box2_max[2]);

    // add the dimensions of the bounding boxes in x and y
    boxes_dim_a = box1_max[0] - box1_min[0] +
      box2_max[0] - box2_min[0];
    boxes_dim_b = box1_max[2] - box1_min[2] +
      box2_max[2] - box2_min[2];
  }
  // check if parallel to y-z-plane and project into y-z-plane
  else if ((box1_max[0] - box1_min[0]) < min_dist_threshold &&
      (box2_max[0] - box2_min[0]) < min_dist_threshold)
  {
    // calculate the distance between the center points (times 2) in x and y
    center_dist_a2 = fabs (box1_min[1] + box1_max[1] -
        box2_min[1] - box2_max[1]);
    center_dist_b2 = fabs (box1_min[2] + box1_max[2] -
        box2_min[2] - box2_max[2]);

    // add the dimensions of the bounding boxes in x and y
    boxes_dim_a = box1_max[1] - box1_min[1] +
      box2_max[1] - box2_min[1];
    boxes_dim_b = box1_max[2] - box1_min[2] +
      box2_max[2] - box2_min[2];
  }
  // 'default' case: projection into x-y-plane
  else
  {
    // calculate the distance between the center points (times 2) in x and y
    center_dist_a2 = fabs (box1_min[0] + box1_max[0] -
        box2_min[0] - box2_max[0]);
    center_dist_b2 = fabs (box1_min[1] + box1_max[1] -
        box2_min[1] - box2_max[1]);

    // add the dimensions of the bounding boxes in x and y
    boxes_dim_a = box1_max[0] - box1_min[0] +
      box2_max[0] - box2_min[0];
    boxes_dim_b = box1_max[1] - box1_min[1] +
      box2_max[1] - box2_min[1];
  }
  // check if the boxes overlap
  if (center_dist_a2 <= boxes_dim_a && center_dist_b2 <= boxes_dim_b)
    return true;
  return false;
}

void
removeOverlapBetweenConcaveHulls (std::vector<CloudVector> &projected_clusters,
    std::vector<CloudVector> &concave_hulls)
{
  // storage for bounding boxes to avoid unnecessary KdTree creation
  std::vector<Eigen::Vector4f> min_boxes;
  std::vector<Eigen::Vector4f> max_boxes;

  Eigen::Vector4f hull_min, hull_max;
  Eigen::Vector4f shift_vec (0.01f, 0.01f, 0.01f, 0.0f);
  Eigen::Vector4f infinity_vec (std::numeric_limits<float>::infinity (),
      std::numeric_limits<float>::infinity (),
      std::numeric_limits<float>::infinity (),
      0.0f);

  std::list<size_t> intersection_indices;
  std::vector<pcl::KdTree<ColorPoint>::Ptr> projected_cluster_trees;
  // allocate variables needed for k-nearest neighbor search (k = 1)
  ColorPoint query_point;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_dists;
  k_indices.resize (1);
  k_sqr_dists.resize (1);
  unsigned int nr_neighbors;
  bool has_closest_point;
  unsigned int nr_points;

  // for each projection plane
  for (size_t i = 0; i < projected_clusters.size (); ++i)
  {
    // clear and resize the bounding boxes
    min_boxes.clear ();
    max_boxes.clear ();
    min_boxes.resize (projected_clusters[i].size ());
    max_boxes.resize (projected_clusters[i].size ());

    // clear and resize the KdTrees
    projected_cluster_trees.clear ();
    projected_cluster_trees.resize (concave_hulls[i].size ());

    // fill min and max boxes
    for (size_t j = 0; j < concave_hulls[i].size (); ++j)
    {
      if (concave_hulls[i][j]->points.size () > 0)
      {
        // get the bounding box corners
        pcl::getMinMax3D (*concave_hulls[i][j], min_boxes[j], max_boxes[j]);
        // slighly increase the bounding boxes
        min_boxes[j] -= shift_vec;
        max_boxes[j] += shift_vec;
      }
      else
      {
        min_boxes[j] = infinity_vec;
        max_boxes[j] = infinity_vec;
      }
    }

    // check for intersectionsof the bounding boxes
    for (size_t j = 0; j < concave_hulls[i].size (); ++j)
    {
      // has the concave hull any points?
      if (concave_hulls[i][j]->points.size () > 0)
      {
        hull_min = min_boxes[j];
        hull_max = max_boxes[j];
        intersection_indices.clear ();
        // check all other bounding boxes for intersections
        for (size_t k = 0; k < concave_hulls[i].size (); ++k)
        {
          if (k != j)
          {
            if (intersectBoundingBoxes (hull_min, hull_max, min_boxes[k],
                  max_boxes[k]))
            {
              intersection_indices.push_back (k);
            }
          }
        }
        // any bounding box intersections?
        if (!intersection_indices.empty ())
        {
          std::list<size_t>::const_iterator it = intersection_indices.begin ();
          // check if needed KdTrees are already created (and create if needed)
          while (it != intersection_indices.end ())
          {
            if (projected_cluster_trees[*it] == NULL)
            {
              pcl::KdTree<ColorPoint>::Ptr kd_tree
                (new pcl::KdTreeFLANN<ColorPoint>);
              kd_tree->setInputCloud (projected_clusters[i][*it]);
              projected_cluster_trees[*it] = kd_tree;
            }
            it++;
          }
          /*
           * now use the KdTrees to decide if points should be removed
           * due to overlap of object clusters
           */
          CloudPtr refined_hull (new Cloud);
          refined_hull->points.reserve (concave_hulls[i][j]->points.size ());
          nr_points = 0;
          for (size_t k = 0; k < concave_hulls[i][j]->points.size (); ++k)
          {
            query_point = concave_hulls[i][j]->points[k];
            nr_neighbors = 0;
            has_closest_point = false;
            it = intersection_indices.begin ();
            while (it != intersection_indices.end ())
            {
              nr_neighbors =
                projected_cluster_trees[*it]->radiusSearch (query_point,
                    MAX_CONC_HULL_CORR_DIST, k_indices, k_sqr_dists, 1);
              if (nr_neighbors > 0)
              {
                has_closest_point = true;
                break;
              }
              it++;
            }
            // point not close to any other projected cluster
            if (!has_closest_point)
            {
              refined_hull->points.push_back (query_point);
              nr_points++;
            }
          }
          refined_hull->points.resize (nr_points);
          refined_hull->height = 1;
          refined_hull->width = nr_points;
          concave_hulls[i][j] = refined_hull;
        }
      }
    }
  }
}

void
colorPointCloud (CloudPtr cloud, uint8_t r, uint8_t g, uint8_t b)
{
  Cloud::VectorType::iterator it;
  it  = cloud->points.begin ();
  while (it != cloud->points.end ())
  {
    it->r = r;
    it->g = g;
    it->b = b;
    it++;
  }
}

void
colorPointCloudVector (CloudVector &cloud_vector, uint8_t r, uint8_t g,
    uint8_t b)
{
  for (size_t i = 0; i < cloud_vector.size (); ++i)
  {
    colorPointCloud (cloud_vector[i], r, g, b);
  }
}

bool
pointInPolygon2D (const std::vector<Eigen::Vector2i> &polygon, const Eigen::Vector2i &query_point)
{
  bool inside = false;

  std::vector<Eigen::Vector2i>::const_iterator start_it, end_it;

  start_it = polygon.end () - 1;  // last vertex
  end_it = polygon.begin ();
  bool start_above, end_above;

  start_above = (*start_it)[1] >= query_point[1] ? true : false;
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

void
createSampleRays (const LabelCloud::ConstPtr &base_cloud, LabelCloudPtr &ray_cloud,
    float sample_dist, Eigen::Vector3f origin)
{
  ray_cloud->points.clear ();

  // get upper limit of number of sample points
  Eigen::Vector3f curr_point;
  LabelPoint min, max, tmp;
  pcl::getMinMax3D (*base_cloud, min, max);
  // create point that is farthest away of bounding box
  Eigen::Vector3f upper_bound (
      (fabs (min.x - origin[0]) > fabs (max.x - origin[0])) ?
      (min.x - origin[0]) : (max.x - origin[0]),
      (fabs (min.y - origin[1]) > fabs (max.y - origin[1])) ?
      (min.y - origin[1]) : (max.y - origin[1]),
      (fabs (min.z - origin[2]) > fabs (max.z - origin[2])) ?
      (min.z - origin[2]) : (max.z - origin[2]));
  double max_dist = sqrt (upper_bound.dot (upper_bound));
  size_t max_sample_points = base_cloud->points.size () * max_dist / sample_dist;
  size_t nr_steps;
  float length;
  // allocate upper bound of memory
  ray_cloud->points.reserve (max_sample_points);

  Eigen::Vector3f ray, step_vec, curr_sample;

  LabelCloud::VectorType::const_iterator p_it = base_cloud->points.begin ();
  while (p_it != base_cloud->points.end ())
  {
    ray = Eigen::Vector3f (p_it->x, p_it->y, p_it->z);
    ray -= origin;
    length = sqrt (ray.dot (ray));
    nr_steps = floor (length / sample_dist);
    step_vec = ray.normalized ();
    step_vec *= -sample_dist;
    curr_sample = ray;

    tmp = *p_it;
    for (size_t i = 0; i < nr_steps; ++i)
    {
      tmp.x = curr_sample[0];
      tmp.y = curr_sample[1];
      tmp.z = curr_sample[2];
      ray_cloud->points.push_back (tmp);
      curr_sample += step_vec;
    }
    p_it++;
  }
  // adapt dimensions of point cloud
  ray_cloud->width = ray_cloud->points.size ();
  ray_cloud->height = 1;
}

// we assume saturation and value to be 1.0f, since we want bright distinguishable colors :)
void
hsv2rgb (float h, float &r, float &g, float &b)
{
  float h_ = h / 60.0f;
  float x, c;
  c = 1.0f;
  x = c * (1.0f - fabs (fmod (h_, 2) - 1));

  if (h_ < .0f)
  {
    ROS_WARN ("hsv2rgb: angle must be positive, provided angle: %f",  h);
    r = g = b = 1.0f;
  }
  else if (0.0f <= h_ && h_ < 1.0f)
  {
    r = c;
    g = x;
    b = 0.0f;
  }
  else if (1.0f <= h_ && h_ < 2.0f)
  {
    r = x;
    g = c;
    b = 0.0f;
  }
  else if (2.0f <= h_ && h_ < 3.0f)
  {
    r = 0.0f;
    g = c;
    b = x;
  }
  else if (3.0f <= h_ && h_ < 4.0f)
  {
    r = 0.0f;
    g = x;
    b = c;
  }
  else if (4.0f <= h_ && h_ < 5.0f)
  {
    r = x;
    g = 0.0f;
    b = c;
  }
  else if (5.0f <= h_ && h_ < 6.0f)
  {
    r = c;
    g = 0.0f;
    b = x;
  }
  else
  {
    ROS_WARN ("Angle must be less than 360°, provided angle: %f", h);
    r = g = b = 1.0f;
  }
}


float
lineSegmentToPointDistance (const Eigen::Vector3f &segment_start, const Eigen::Vector3f &segment_end,
    const Eigen::Vector3f query_point)
{
  Eigen::Vector3f segment_base = segment_end - segment_start;
  Eigen::Vector3f start_to_query = query_point - segment_start;

  float cos_segment_start_to_query, cos_segment_end_to_query;

  // get the cosine between the vector start->end and start->query_point
  cos_segment_start_to_query = start_to_query.dot (segment_base);
  // check if angle between line segment and start->query_point is larger than 90°
  if (cos_segment_start_to_query <= 0)  // query point is before segment start
  {
    // return distance between query point and segment start
    return sqrt (start_to_query.dot (start_to_query));
  }
  // get the cosine between (extension of) line segment and end->query_point
  cos_segment_end_to_query = segment_base.dot (segment_base);
  // check if angle between (extension of) line segment and end->query is smaller than 90°
  if (cos_segment_end_to_query <= cos_segment_start_to_query) // query point is after segment end
  {
    return sqrt ((query_point - segment_end).dot (query_point - segment_end));
  }
  float tmp = cos_segment_start_to_query / cos_segment_end_to_query;

  // get the projection of the query point onto line segment
  Eigen::Vector3f proj = segment_start + tmp * segment_base;

  // return distance between query point and its projection
  return sqrt ((query_point - proj).dot (query_point - proj));
}

float
convexHullsMinDistance (const std::vector<Eigen::Vector3f> &convex_hull_a,
    std::vector<Eigen::Vector3f> &convex_hull_b)
{
  // TODO: remove later
  std::string red_start = "\033[1;31m";
  std::string red_end = "\033[0m";

  // get centroids
  Eigen::Vector3f centroid_a = getCentroid (convex_hull_a);
  Eigen::Vector3f centroid_b = getCentroid (convex_hull_b);
  std::vector<Eigen::Vector3f>::const_iterator hull_it;

  Eigen::Hyperplane<float, 3> hyperplane_a ((centroid_b - centroid_a).normalized (), centroid_a);
  Eigen::Hyperplane<float, 3> hyperplane_b ((centroid_a - centroid_b).normalized (), centroid_b);

  // gather relevant points for minimal distance check
  std::vector<Eigen::Vector3f> check_set_a, check_set_b;
  getPointsOnPositiveSideOfHyperplane (convex_hull_a, hyperplane_a, check_set_a);
  getPointsOnPositiveSideOfHyperplane (convex_hull_b, hyperplane_b, check_set_b);

  /*
  std::cout << "check_set_b:" << std::endl;
  for (size_t i = 0; i < check_set_b.size (); ++i)
  {
    std::cout << check_set_b[i][0] << ", " << check_set_b[i][1] << ", " << check_set_b[i][2] << std::endl;
  }
  */
  float min_dist = std::numeric_limits<float>::max ();

  // check the distances between all points in check_set_a and the line segments constructed by check_set_b
  // since we don't modify the order of the points in the convex hulls all valid line segments should still
  // be retained and one (artificial) new line segment with a (larger) distance should be added that we don't
  // need to handle explicitly

  hull_it = check_set_a.begin ();
  std::vector<Eigen::Vector3f>::const_iterator segment_start_it, segment_end_it;
  float curr_dist;
  std::vector<Eigen::Vector3f>::const_iterator a, b, c;

  //std::cout << red_start << "first segement start:\n " << *segment_start_it << std::endl;
  //std::cout << "first segement end:\n " << *segment_end_it << red_end << std::endl;
  while (hull_it != check_set_a.end ())
  {
    segment_start_it = check_set_b.begin ();
    segment_end_it = --check_set_b.end ();
    while (segment_start_it != check_set_b.end ())
    {
      curr_dist = lineSegmentToPointDistance (*segment_start_it, *segment_end_it, *hull_it);
      if (curr_dist < min_dist)
      {
        min_dist = curr_dist;
        a = segment_start_it;
        b = segment_end_it;
        c = hull_it;
      }
      segment_end_it = segment_start_it++;
    }
    hull_it++;
  }
  hull_it = check_set_b.begin ();
  while (hull_it != check_set_b.end ())
  {
    segment_start_it = check_set_a.begin ();
    segment_end_it = --check_set_a.end ();
    while (segment_start_it != check_set_a.end ())
    {
      curr_dist = lineSegmentToPointDistance (*segment_start_it, *segment_end_it, *hull_it);
      if (curr_dist < min_dist)
      {
        min_dist = curr_dist;
        a = segment_start_it;
        b = segment_end_it;
        c = hull_it;
      }
      segment_end_it = segment_start_it++;
    }
    hull_it++;
  }

  std::cout << red_start << "segment_start: " << (*a)[0] << ", " << (*a)[1] << ", " << (*a)[2] << std::endl;
  std::cout << "segment_end: " << (*b)[0] << ", " << (*b)[1] << ", " << (*b)[2] << std::endl;
  std::cout << "query_point " << (*c)[0] << ", " << (*c)[1] << ", " << (*c)[2] << red_end << std::endl;

  return min_dist;
}

bool
convexHullDistBelowThreshold (const std::vector<Eigen::Vector3f> &convex_hull_a,
    std::vector<Eigen::Vector3f> &convex_hull_b, float threshold)
{
  Eigen::Vector3f centroid_a = getCentroid (convex_hull_a);
  Eigen::Vector3f centroid_b = getCentroid (convex_hull_b);

  Eigen::Hyperplane<float, 3> hyperplane_a ((centroid_b - centroid_a).normalized (), centroid_a);
  Eigen::Hyperplane<float, 3> hyperplane_b ((centroid_a - centroid_b).normalized (), centroid_b);

  // gather relevant points for minimal distance check
  std::vector<Eigen::Vector3f> check_set_a, check_set_b;
  getPointsOnPositiveSideOfHyperplane (convex_hull_a, hyperplane_a, check_set_a);
  getPointsOnPositiveSideOfHyperplane (convex_hull_b, hyperplane_b, check_set_b);

  // start to iterator over check_set_a
  std::vector<Eigen::Vector3f>::const_iterator hull_it, segment_start_it, segment_end_it;
  hull_it = check_set_a.begin ();
  while (hull_it != check_set_a.end ())
  {
    segment_start_it = check_set_b.begin ();
    segment_end_it = --check_set_b.end ();
    while (segment_start_it != check_set_b.end ())
    {
      if (lineSegmentToPointDistance (*segment_start_it, *segment_end_it, *hull_it) < threshold)
        return true;
      segment_end_it = segment_start_it++;
    }
    hull_it++;
  }
  hull_it = check_set_b.begin ();
  while (hull_it != check_set_b.end ())
  {
    segment_start_it = check_set_a.begin ();
    segment_end_it = --check_set_a.end ();
    while (segment_start_it != check_set_a.end ())
    {
      if (lineSegmentToPointDistance (*segment_start_it, *segment_end_it, *hull_it) < threshold)
        return true;
      segment_end_it = segment_start_it++;
    }
    hull_it++;
  }
  return false;
}

Eigen::Vector3f getCentroid (const std::vector<Eigen::Vector3f> &vec)
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero ();

  std::vector<Eigen::Vector3f>::const_iterator vec_it;
  vec_it = vec.begin ();
  while (vec_it != vec.end ())
  {
    center += (*vec_it++);
  }
  center /= static_cast<float> (vec.size ());
  return center;
}

void getPointsOnPositiveSideOfHyperplane (const std::vector<Eigen::Vector3f> &all_points,
    const Eigen::Hyperplane<float, 3> &hyperplane, std::vector<Eigen::Vector3f> &points_on_pos_side)
{
  points_on_pos_side.clear ();
  points_on_pos_side.reserve (all_points.size ());

  std::vector<Eigen::Vector3f>::const_iterator point_it;
  point_it = all_points.begin ();
  while (point_it != all_points.end ())
  {
    if (hyperplane.signedDistance (*point_it) >= 0.0f)
    {
      points_on_pos_side.push_back (*point_it);
    }
    point_it++;
  }
}

bool
doConvexHulls2DIntersect (const std::vector<Eigen::Vector3f> &convex_hull_a,
    const std::vector<Eigen::Vector3f> &convex_hull_b)
{
  Eigen::Vector3f centroid_a = getCentroid (convex_hull_a);
  Eigen::Vector3f centroid_b = getCentroid (convex_hull_b);

  Eigen::Hyperplane<float, 3> hyperplane_a ((centroid_b - centroid_a).normalized (), centroid_a);
  Eigen::Hyperplane<float, 3> hyperplane_b ((centroid_a - centroid_b).normalized (), centroid_b);

  // gather relevant points for minimal distance check
  std::vector<Eigen::Vector3f> check_set_a, check_set_b;
  getPointsOnPositiveSideOfHyperplane (convex_hull_a, hyperplane_a, check_set_a);
  getPointsOnPositiveSideOfHyperplane (convex_hull_b, hyperplane_b, check_set_b);


  std::vector<Eigen::Vector3f>::const_iterator hull_it;
  hull_it = check_set_a.begin ();
  while (hull_it != check_set_a.end ())
  {
    if (pointInsideConvexPolygon (convex_hull_b, *hull_it++))
    {
      return true;
    }
  }
  hull_it  = check_set_b.begin ();
  while (hull_it != check_set_b.end ())
  {
    if (pointInsideConvexPolygon (convex_hull_a, *hull_it++))
    {
      return true;
    }
  }
  return false;
}

void
getBBox (const std::vector<Eigen::Vector2i> &border, Eigen::Vector2i &min_bbox, Eigen::Vector2i &max_bbox)
{
  min_bbox[0] = min_bbox[1] = std::numeric_limits<int>::max ();
  max_bbox[0] = max_bbox[1] = std::numeric_limits<int>::min ();

  //std::cout << "initial min: " << min_bbox[0] << ", " << min_bbox[1] << std::endl;
  //std::cout << "initial max: " << max_bbox[0] << ", " << max_bbox[1] << std::endl;

  std::vector<Eigen::Vector2i>::const_iterator b_it = border.begin ();
  while (b_it != border.end ())
  {
    //std::cout << "\t\t" << (*b_it)[0] << ", " << (*b_it)[1] << std::endl;
    if ((*b_it)[0] < min_bbox[0])
    {
      min_bbox[0] = (*b_it)[0];
    }
    if ((*b_it)[1] < min_bbox[1])
    {
      min_bbox[1] = (*b_it)[1];
    }
    if ((*b_it)[0] > max_bbox[0])
    {
      max_bbox[0] = (*b_it)[0];
    }
    if ((*b_it)[1] > max_bbox[1])
    {
      max_bbox[1] = (*b_it)[1];
    }
    b_it++;
  }
  //std::cout << "final min: " << min_bbox[0] << ", " << min_bbox[1] << std::endl;
  //std::cout << "final max: " << max_bbox[0] << ", " << max_bbox[1] << std::endl;
}

// TODO: optional argument to change description comment in image header, if set
void storeAsImage (const std::vector<std::vector<Eigen::Vector3i> > &img_data, const std::string &filename)
{
  if (img_data.size () == 0)
  {
    std::cerr << "storeAsImage was provided with empty data; Exiting" << std::endl;
    return;
  }

  std::ofstream img_file (filename.c_str ());
  img_file << "P3\n" << "# general ppm image\n" << img_data.size () << " "
        << img_data[0].size () << "\n255" << std::endl;

  for (size_t v = 0; v < img_data[0].size (); ++v)
  {
    for (size_t u = 0; u < img_data.size (); ++u)
    {
      img_file << img_data[u][v][0] << " " << img_data[u][v][1] << " " << img_data[u][v][2] << std::endl;
    }
  }
  img_file.flush ();
  img_file.close ();
}

transparent_object_reconstruction::ViewpointInterval
convertICLInterval2ViewpointInterval (const boost::icl::discrete_interval<int> &icl_interval)
{
  transparent_object_reconstruction::ViewpointInterval vpi;
  vpi.lower = icl_interval.lower ();
  vpi.upper = icl_interval.upper ();
  return vpi;
}

boost::icl::discrete_interval<int>
convertViewpointInterval2ICLInterval (const transparent_object_reconstruction::ViewpointInterval &vpi)
{
  return boost::icl::construct<boost::icl::discrete_interval<int> >
    (vpi.lower, vpi.upper, boost::icl::interval_bounds::closed ());
}

void
convertICLIntervalSet2VoxelViewpointIntervals (const boost::icl::interval_set<int> &icl_interval_set,
    transparent_object_reconstruction::VoxelViewPointIntervals &voxel_vp_intervals)
{
  voxel_vp_intervals.intervals.clear ();
  voxel_vp_intervals.intervals.reserve (icl_interval_set.iterative_size ());
  boost::icl::interval_set<int>::const_iterator interval_it = icl_interval_set.begin ();
  while (interval_it != icl_interval_set.end ())
  {
    voxel_vp_intervals.intervals.push_back (convertICLInterval2ViewpointInterval (*interval_it));
    interval_it++;
  }
}

void
convertVoxelViewpointIntervals2ICLIntervalSet (const transparent_object_reconstruction::VoxelViewPointIntervals &voxel_vp_intervals,
    boost::icl::interval_set<int> &icl_interval_set)
{
  icl_interval_set.clear ();
  std::vector<transparent_object_reconstruction::ViewpointInterval>::const_iterator interval_it = voxel_vp_intervals.intervals.begin ();
  while (interval_it != voxel_vp_intervals.intervals.end ())
  {
    icl_interval_set.insert (convertViewpointInterval2ICLInterval (*interval_it));
    interval_it++;
  }
}
