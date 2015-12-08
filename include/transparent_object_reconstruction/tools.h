#ifndef TRANSP_OBJ_RECON_TOOLS
#define TRANSP_OBJ_RECON_TOOLS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/Mesh.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/concave_hull.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>

#include <transparent_object_reconstruction/common_typedefs.h>

// collection of methods used to transform point clouds into x-y-plane etc. pp.
typedef pcl::ModelCoefficients Model;
typedef Model::Ptr ModelPtr;
typedef std::vector<CloudPtr> CloudVector;
typedef std::vector<ModelPtr> ModelVector;
typedef std::vector<std::vector<Eigen::Vector4f> > TransPlaneCoeffsVec;

// TODO: the following is copied, refine later

// plane distance threshold
const double PLANE_INLIER_THRESHOLD = 0.015; // 1.5cm

// threshold for checking for another plane in the scene
const double MIN_PLANE_CHECK_INLIER_RATIO = 0.25;
const size_t MIN_NR_PLANE_INLIERS = 15000;

// euclidean cluster thresholds
const double CLUSTER_TOLERANCE = 0.02; // 2cm
const unsigned int MIN_CLUSTER_SIZE = 500;
const unsigned int MAX_CLUSTER_SIZE = 20000;

// eps for line with plane intersection
const double LINE_PLANE_ANGLE_EPS = 1e-3;

// threshold for closest point plane <-> concave hull
// TODO: threshold could be made accesible via command line
const double MAX_CORR_DIST = 0.005; // 0.5cm
const double MAX_SQRD_CORR_DIST = MAX_CORR_DIST * MAX_CORR_DIST;
const float MIN_CORRELATION_RATIO = 0.5f;

// threshold for point removal from concave hulls due to potential
// overlap with other hulls
const float MAX_CONC_HULL_CORR_DIST = 0.02f;

const float CONCAVE_HULL_ALPHA = 0.005f;
// TODO: end of blindly importet constants

/**
 * Convenience template method to convert the points in a given point cloud into a vector
 * of 'Eigen::Vector3f' just representing the geometric properties of the point cloud.
 */
template <typename PointT> inline void
convert (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, std::vector<Eigen::Vector3f> &vec)
{
  vec.clear ();
  vec.reserve (cloud->points.size ());
  typename pcl::PointCloud<PointT>::VectorType::const_iterator p_it = cloud->points.begin ();
  while (p_it != cloud->points.end ())
  {
    vec.push_back (Eigen::Vector3f (p_it->x, p_it->y, p_it->z));
    p_it++;
  }
}

/**
 * Function to retrieve the points of a point cloud that fall into each
 * grid cell of a voxel grid. The returned 'std::vector<Cloud::VectorType>'
 * stores in each 'Cloud::VectorType' the points from point cloud 'cloud' that
 * have the same centroid index in voxel grid 'v_grid'.
 *
 * @param[in] cloud Cloud to contain the original points
 * @param[in] v_grid A voxel grid representation of the point cloud
 * @param[in] nr_filled_grid_cells Specifies the number of occupied grid
 *   cells in voxelgrid 'v_grid'
 * @param[out] cell_points Vector of point vectors - each point vector
 *   contains the points that are present in one specific grid cell
 **/
void
extractVoxelGridCellPoints (const CloudPtr &cloud,
    pcl::VoxelGrid<ColorPoint> &v_grid,
    size_t nr_filled_grid_cells,
    std::vector<Cloud::VectorType> &cell_points);


/**
 * Method to filter grid cells according to the number of points in
 * each cell.
 *
 * @param[in] cell_points A vector of 'Cloud::VectorType' containing the
 *   points inside each single grid cell
 * @param[in] min_points_in_cell Threshold denoting the minimal number
 *   of points to consider a grid cell as occupied
 * @param[out] indices The indices of the grid cells that contain at
 * least 'min_points_in_cell' points
 **/
void
filterGridCells (const std::vector<Cloud::VectorType> &cell_points,
    size_t min_points_in_cell, std::vector<size_t> &indices);

/**
 * Simple comparator to induce ordering on point clouds based on their
 * size. Size here refers to the number of contained points, not to the
 * dimensions (volume) of the point clouds.
 *
 * @param[in] a The first cloud to be compared
 * @param[in] b The second cloud to be compared
 * @returns True if cloud 'a' contains more points than 'b';
 *   false otherwise
 **/
bool
sortCloudBySizeDesc (const CloudPtr &a, const CloudPtr &b);

/**
 * @brief method to align the plane coefficients so that the kinect
 * (i.e. the origin) lies above the plane.
 * If the origin lies above the given plane coefficients these will not
 * be modified. However if the origin lies below the plane, all plane
 * coefficients will be multiplied by -1 (thus inverting the normal
 * direction while retaining the same plane.
 *
 * @param plane_coefficients the coefficients of the plane as a
 * pcl::ModelCoefficients::Ptr
 *
 * @return true if the plane coefficients were modified, false otherwise
 **/
bool
alignPlaneCoefficientsToOrigin (ModelPtr plane_coefficients);


void
calcPlaneTransformation (Eigen::Vector3f plane_normal,
    const Eigen::Vector3f &origin, Eigen::Affine3f &transformation);



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
    std::vector<Eigen::Affine3f> &transformation_vector);

/**
 * Method to transform several plane equations according to given affine
 * transformations. All plane equations are transformed by each given
 * transformation, given via input argument 'transformation_vector'. Plane
 * equations that were transformed by the same affine transformation are
 * stored in a vector. All these vectors (number depends on the size of
 * 'transformation_vector' are in turn stored and returned via output
 * argument 'transformed_plane_equations'. The original untransformed
 * plane equations are provided by input argument 'plane_coeff_vector'.
 *
 * @param[in] plane_coeff_vector A vector containing all plane equations
 * @param[in] transformation_vector A vector containing affine
 *    transformations
 * @param[out] transformed_plane_equations A vector containing vectors of
 *    transformed plane equations
 **/
void
calcTransformedPlaneEquations (const ModelVector &plane_coeff_vector,
    const std::vector<Eigen::Affine3f> &transformation_vector,
    TransPlaneCoeffsVec &transformed_plane_equations);

/**
 * Method to crop the inliers of a given plane by other plane equations.
 * That means that each inlier is checked if it lies behind one of the
 * other planes (and could thus be the result of a measurement error).
 *
 * @param[out] cropped_plane The remaining inliers after being cropped by
 *    the other planes.
 * @param[in] plane_vector Vector of point clouds with all inliers for
 *    available planes.
 * @param[in] plane_coeff_vector Vector of plane model coefficients for
 *    all planes (indices have to correspond to 'plane_vector').
 * @param[in] plane_index Index to specify which plane inliers are going
 *    to be cropped by the other planes.
 **/
void
cropPlaneByPlanes (CloudPtr cropped_plane, const CloudVector &plane_vector,
    const ModelVector &plane_coeff_vector, size_t plane_index);

/**
 * Method to crop the inliers of a given plane by other plane equations.
 * That means that each inlier is checked if it lies behind one of the
 * other planes (and could thus be the result of a measurement error).
 * Similar to the previous method, but this time inliers will be passed
 * along via argument 'plane_vector' which will also serves as the output
 * argument, i.e. the plane inliers will eventually be modified (i.e. some
 * may be deleted) in this version and the original input is not retained.
 * The index specified via 'plane_index' determines which of the point
 * clouds contained in 'plane_vector' will be modified.
 *
 * @param[in,out] plane_vector Vector of point clouds containing plane inliers
 * @param[in] plane_coeff_vector Model coefficients for all planes
 * @param[in] plane_index Index of the plane to be cropped
 **/
void
cropPlaneByPlanes (CloudVector &plane_vector,
    const ModelVector &plane_coeff_vector, size_t plane_index);

/**
 * Crops all points in point cloud 'cloud' by all planes given via
 * ModelVector 'plane_coeff_vector'. Notice that cloud can be modified
 * by this method, i.e., all points that (looking from the origin) lie
 * behind any of the given planes will be removed from the point cloud.
 *
 * @param[in,out] cloud The point cloud that will be cropped.
 * @param[in] plane_coeff_vector A Vector containing all planes that will
 *   be used to crop the point cloud.
 **/
void
cropPointCloudByAllPlanes (CloudPtr cloud, const ModelVector &plane_coeff_vector);

/**
 * Crops all points in point cloud 'cloud' by all planes given via
 * ModelVector 'plane_coeff_vector'. Notice that cloud can be modified
 * by this method, i.e., all points that (looking from the origin) lie
 * behind any of the given planes will be removed from the point cloud.
 * The third argument can be used to specify a single plane that will not
 * be used in the cropping process.
 *
 * @param[in,out] cloud The point cloud that will be cropped.
 * @param[in] plane_coeff_vector A Vector containing all planes that will
 *    be used to crop the point cloud.
 * @param[in] not_used_plane_index Index of a single plane that will
 *    not be used for cropping.
 **/
void
cropPointCloudByPlanes (CloudPtr cloud,
    const ModelVector &plane_coeff_vector,
    size_t not_used_plane_index);

/**
 * Crops all points in point cloud 'cloud' by all specified planes given
 * via ModelVector 'plane_coeff_vector' and index vector 'crop_indices'.
 * Notice that cloud can be modified by this method, i.e., all points that
 * (looking from the origin) lie behind any of the given planes will be
 * removed from the point cloud.
 *
 * @param[in,out] cloud The point cloud that will be cropped.
 * @param[in] plane_coeff_vector A Vector containing all planes that will
 *    be used to crop the point cloud.
 * @param[in] crop_indices The indices of the planes that will be used in
 *    the cropping process.
 **/
void
cropPointCloudBySpecifiedPlanes (CloudPtr cloud,
    const ModelVector &plane_coeff_vector,
    const std::vector<size_t> &crop_indices);

/**
 * Function to check if a given point lies inside a given convex(!)
 * polygon. It is assumed that point and polygon lie in the same plane,
 * i.e., although we are using 3D points this is actually a 2D problem.
 * The polygon is defined via its vertices that are passed as the points
 * of the point cloud input argument. Note that this method does neither
 * test the polygon for convexity nor for planarity or if point and
 * polygon lie in the same plane. If any of these conditions is not met
 * the returned results might not correspond with expectations.
 *
 * @param[in] polygon The polygon given as a std::vector of Eigen::Vector3f
 * @param[in] query_point The query point that will be checked.
 * @return true if the query point lies inside the polygon;
 *   false otherwise
 **/
bool
pointInsideConvexPolygon (const std::vector<Eigen::Vector3f> &polygon, const Eigen::Vector3f &query_point);

/**
  * Templated version for point clouds of function 'pointInsideConvexPolygon ()'.
  * Internally converts the point cloud to a std::vector<Eigen::Vector3f> and calls the
  * non-templated version.
  */
template <typename PointT> inline bool
pointInsideConvexPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &polygon, PointT query_point)
{
  std::vector<Eigen::Vector3f> p;
  convert<PointT> (polygon, p);
  Eigen::Vector3f q (query_point.x, query_point.y, query_point.z);

  return pointInsideConvexPolygon (p, q);
}

/**
  * Method to filter points of a projected cluster hull via a polygon.
  * The first argument 'plane_convex_hull' should be the extracted convex
  * hull of a filtered plane, i.e., it is essentially a planar polygon
  * containing all plane inliers. The second argument should be the
  * projected convex hull of a cluster. That means that first clusters need
  * to be extracted and the contour of them needs to be computed. Such
  * a convex hull is then projected onto the same plane of which the convex
  * hull was passed (first argument). A projected cluster contour that fits
  * these requirements should be passed as the second argument. The output
  * will consist of all projected points of the cluster contour that lie
  * inside the convex hull of the plane.
  * Note that the test if a projected cluster contour point lies inside the
  * convex hull is performed by 'pointInsideConvexPolygon ()'.
  *
  * @param[in] plane_convex_hull The convex hull of a plane
  * @param[in] proj_cluster_hull The points of a cluster hull, projected
  *   into the plane.
  * @param[out] filtered_cluster_hull The points of the projected cluster
  *   hull that are inside the convex hull
  **/
void
cropClusterHullByPlaneHull (Cloud::ConstPtr plane_convex_hull,
    Cloud::ConstPtr proj_cluster_hull, Cloud &filtered_cluster_hull);

/**
 * Function to compute the intersection of line with a plane.
 * The line is specified by two points that lie on the line,
 * passed via arguments  'point_a' and 'point_b', while the plane
 * is specified via argument 'plane'. The return value of the function
 * indicates if there is an intersection. If the line intersects the
 * plane then the point of intersection is passed via output argument
 * 'point'. Note that in the case of no intersection 'point' will contain
 * the coordinates (0,0,0,0).
 * The last argument 'angle_eps' is a threshold to determine
 * if line in plane are parallel (i.e., the line direction is orthogonal
 * to the plane normal) and therefore never intersect.
 *
 * @param[in] point_a The first point on the line
 * @param[in] point_b The second point on the line
 * @param[in] plane The coefficients of the plane
 * @param[out] point The point of the intersection between line and plane
 * @param[in] angle_eps Threshold to check for parallelity
 * @return true if the line intersects the plane; false otherwise
 **/
bool 
lineWithPlaneIntersection (const Eigen::Vector3f point_a,
    const Eigen::Vector3f point_b, const Eigen::Vector4f &plane,
    Eigen::Vector4f &point, double angle_eps);

/**
 * Method to refine the inliers of several (at least 2) planes.
 * Regular plane extraction uses a distance threshold for plane
 * association. If performed iteratively to detect planes of decreasing
 * point size (which is commonly done) points that are near the
 * intersection of 2 planes are not necessarily associated with the plane
 * they fit best into, but with the plane that was detected first. This
 * method checks for each inlier point to which plane it is closest to
 * (by the provided plane coefficients) and then associates this inlier
 * to closest plane. The refined planes are returned via the same argument
 * as they are provided ('planes'), so be aware that the input will be
 * modified.
 *
 * @param[in,out] planes The point inliers of all planes that will be
 *   refined
 * @param[in] plane_coefficients The coefficients for all planes
 **/
void
refinePlanes (CloudVector &planes, ModelVector &plane_coefficients);

/**
 * Method to project the points of a point cloud onto a given plane.
 *
 * @param[in] input The cloud that will be projected
 * @param[out] projected_cloud The projected cloud
 * @param[in] plane The plane onto which the points of 'cloud' are
 *    projected onto
 **/
void 
projectCloudOnPlane (Cloud::ConstPtr input, CloudPtr projected_cloud, ModelPtr plane);

/**
 * Wrapper method to create Euclidean clusters of a given point cloud.
 * Clusters are returned as a vector of point clouds. The input point
 * cloud should usually consist of an already filtered point cloud without
 * large planar surfaces etc. This method is intended to generate clusters
 * of the remaining points after such a filtering step.
 *
 * @param[in] input The input point cloud
 * @param[out] output The detected clusters as a vector of point clouds
 * @param[in] max_cluster_size Threshold for the maximal number of points
 *    in a cluster
 * @param[in] min_cluster_size Threshold for the minimal number of points
 *    in a cluster
 * @param[in] cluster_tolerance Distance threshold to determine when points
 *    are still assumed to belong to the same cluster
 **/
void
createEuclideanClusters (Cloud::ConstPtr input, CloudVector &output,
   unsigned int max_cluster_size,
   unsigned int min_cluster_size,
   double cluster_tolerance);
 
/**
 * Method wrapper to project a point cloud into a given plane,
 * using 'pcl::ProjectInliers'. Notice that the result will usually
 * differ from 'projectCloudOnPlane ()', which usually yields theh
 * preferable results in this application case.
 *
 * @param[in] input The point cloud to be projected onto the plane.
 * @param[out] projected_cloud The cloud projected onto the plane
 * @param[in] plane_coefficients The coefficients specifying the plane.
 **/
void
projectCloudToPlane (Cloud::ConstPtr input, CloudPtr projected_cloud,
    ModelPtr plane_coefficients);

/**
 * @brief: Templated method to create a point sampling of the border of a
 * hull polygon.
 * The point cloud given by first argument 'hull' should contain the
 * vertices of a hull polygon. It is assumed that this polygon is closed,
 * i.e., the first and last point are also connected via a line segment.
 * The method proceeds to sample the line segments with a step length as
 * specified by argument 'step_length'.
 *
 * @param[in] hull The vertices of a hull polygon
 * @param[out] hull_lines A point sampling of the line segments, connecting
 *    the vertices of 'hull', also including the vertices themselves.
 * @param[in] step_length The step length of of the sampling
 **/
template <typename PointT> inline void
createHullLines (const typename pcl::PointCloud<PointT>::Ptr &hull,
    typename pcl::PointCloud<PointT>::Ptr hull_lines, float step_length)
{
  // remove any lingering content in the output
  hull_lines->points.clear ();
  // create variables for line computation
  Eigen::Vector3f line_start, line_end, line_direction, line_step;
  PointT point_on_line;
  float line_length;
  unsigned int nr_steps;

  for (size_t point_index = 0; point_index < hull->points.size ();
      ++point_index)
  {
    // set start and end point of current line segment
    if (point_index == 0)
    {
      point_on_line = hull->points.back ();
    }
    else
    {
      point_on_line = hull->points[point_index - 1];
    }
    line_start = Eigen::Vector3f (point_on_line.x, point_on_line.y,
        point_on_line.z);
    point_on_line = hull->points[point_index];
    line_end = Eigen::Vector3f (point_on_line.x, point_on_line.y,
        point_on_line.z);
    // determine direction, length and nr of steps
    line_direction = line_end - line_start;
    line_length = sqrt (line_direction.dot (line_direction));
    line_direction.normalize ();
    line_step = line_direction * step_length;
    nr_steps = line_length / step_length;

    // reserve additional space in the vector
    hull_lines->points.reserve (hull_lines->points.size () + nr_steps + 1);

    // add the sampled line segment
    for (unsigned int step = 0; step < nr_steps; ++step)
    {
      line_start += line_step;
      point_on_line.x = line_start[0];
      point_on_line.y = line_start[1];
      point_on_line.z = line_start[2];
      hull_lines->push_back (point_on_line); // slightly inefficient
    }
    hull_lines->push_back (hull->points[0]);
  }
  // adapt cloud dimensions
  hull_lines->width = hull_lines->points.size ();
  hull_lines->height = 1;
}


/**
 * Method to create a line segment sampling for a collection of hull
 * polygons given via a point cloud vector. Each hull polygon is sampled
 * using method 'createHullLines ()'.
 *
 * @param[in] hull_vector A vector of hull polygons
 * @param[out] hull_lines_vec A vector containing the sampled line
 *   segments for each hull polygon
 * @param step_length The step length for the sampling
 **/
void
createHullLinesVec (CloudVector &hull_vector, CloudVector &hull_lines_vec,
    float step_length);

/**
 * Conveniencen method to create a vector of pcl::VoxelGrid for a number
 * of point cloud passed via vector 'cloud_vector'. The created grids are
 * stored in output argument 'v_grid_vector' while the filtered pointi
 * clouds are returned via output argument 'grid_clouds'.
 *
 * @param[in] cloud_vector
 * @param[out] v_grid_vector
 * @param[out] grid_clouds
 * @param[in] x_grid_size The dimension of a grid cell along th x-axis
 * @param[in] y_grid_size The dimension of a grid cell along th y-axis
 * @param[in] z_grid_size The dimension of a grid cell along th z-axis
 **/
void
createVoxelGrids (CloudVector &cloud_vector,
    std::vector<pcl::VoxelGrid<ColorPoint> > &v_grid_vector,
    CloudVector &grid_clouds,
    float x_grid_size,
    float y_grid_size,
    float z_grid_size);

/**
 * Convenience function for plane extraction via pcl::SACSegmentation.
 * Detects via RANSAC a plane in the point cloud given via 'input' and
 * returns the inliers via output argument 'plane_cloud'. Non-inliers
 * are returned via 'non_planar_cloud' and the plane coefficients via
 * output argument 'plane_coefficients'. The functions furthermore returns
 * the number of inliers in the detected plane.
 *
 * @param[in] input
 * @param[out] non_planar_cloud
 * @param[out] plane_cloud
 * @param[out] plane_coefficients
 * @return The number of inliers in the detected plane
 **/
size_t
plane (Cloud::ConstPtr input, CloudPtr &non_planar_cloud,
    CloudPtr &plane_cloud, ModelPtr plane_coefficients);

/**
 * Method to calculate the hulls of several 'perfect' planes, provided
 * as a point cloud vector via the input argument 'perfect_planes_vector'.
 * The planes (or more correctly planar surfaces) consist of their
 * inliers. The 'perfect' hints that the inliers should have been
 * preprocessed by prjecting them into the plane (for example by
 * previously calling 'projectCloudOnPlane ()'. The corresponding planar
 * convex hulls are returned as their vertices via output argument
 * 'conv_hulls'.
 *
 * @param[in] perfect_planes_vector
 * @param[out] conv_hulls
 **/
void
calcConvexHullsPerfectPlanes (CloudVector &perfect_planes_vector,
    CloudVector &conv_hulls);

/**
 * Convenience method for the computation of the convex hulls for
 * several planar surfaces given as a vector of point clouds. Each
 * planar surface is represented by its inlying measurement points.
 * The convex hulls will be denoted by the vertices of the polygon.
 * Since the convex hull computation in pcl sometimes has / had problems
 * with non planar surfaces, the inlier points are first projected onto
 * their corresponding planes (provided via input argument
 * 'plane_coeff_vector') and calls of 'projectCloudToPlane ()' and
 * afterwards 'calcConvexHullsPerfectPlanes ()' is called to actually
 * compute the convex hulls.
 *
 * @param[in] plane_vector A vector containing inliers of planar surfaces
 * @param[out] conv_hulls A vector containing the convex hulls of the
 *    planar surfaces
 * @param[in] plane_coeff_vector A vector containing the model coefficients
 *    for the planar surfaces.
 **/
void
calcConvexHulls (CloudVector &plane_vector, CloudVector &conv_hulls,
    ModelVector &plane_coeff_vector);

/**
 * Function to remove points with invalid 3D coordinates from a
 * given point cloud.
 *
 * @param[in] input The original point cloud, potentially containing
 *   points with nan-value measurements
 * @param[out] cloud_without_nans The filtered output cloud without
 *   any nan-value points.
 * @return The number of points contained in the filtered cloud
 **/
size_t
removeNANs (Cloud::ConstPtr input, CloudPtr &cloud_without_nans);

/**
 * Method to create 'concave hulls' for number of object clusters. The term
 * 'concave hulls' is due to the used 'pcl::ConcaveHull<>' class and
 * basically describes a contour following of an object cluster, or more
 * precisely, its projection onto a given plane. Object clusters are given
 * via input argument 'object_clusters', while the planes onto which the
 * clusters are projected are passed by 'plane_coefficients'. Via the
 * affine transformations provided by 'plane_transformations' each
 * projected cluster is transformed into the x-y-plane, where the actual
 * contour following takes place (at least in old pcl versions
 * 'pcl::ConcaveHull<>' sometimes returned unexpected results for inputs
 * that were not aligned with the x-y-plane). Optional input parameter
 * 'crop_by_other_planes' determines if extracted contour lines can be
 * cropped by other planes, if the contours are located near the edges of
 * a planar surface. As output the method delivers the projected clusters
 * via 'projected_clusters' and the concave hulls via 'concave_hulls'.
 * Notice that each object cluster is projected into every plane, and the
 * first index of the returned vectors indicates onto which plane a cluster
 * was projected on, while the second index corresponds with the object's
 * index in vector 'object_clusters'.
 *
 * @param[in] object_clusters A vector containing object clusters as
 *    point clouds
 * @param[in] plane_coefficients A vector containing coefficients of
 *    detected planar surfaces
 * @param[in] plane_transformations A vector containing transformations to
 *    transform the planar surfaces onto the x-y-plane
 * @param[out] projected_clusters A vector of vectors containing the
 *    object clusters projected onto the different planes
 * @param[out] concave_hulls A vector of vectors containing the contour
 *    lines of the projected object clusters
 * @param[in] crop_by_other_planes Argument to determine if the computed
 *    concave hulls are to be cropped by intersecting planes
 **/
void
createConcaveHulls (CloudVector &object_clusters,
    ModelVector &plane_coefficients,
    std::vector<Eigen::Affine3f> &plane_transformations,
    std::vector<CloudVector> &projected_clusters,
    std::vector<CloudVector> &concave_hulls,
    bool crop_by_other_planes);

/**
 * Method to crop existing contour lines of projected object clusters by
 * the convex hull describing a planar surface.
 *
 * @param[in] convex_hulls A vector containing convex hulls for planar
 *   surfaces
 * @param[in,out] A vector of vectors containing the concave hulls;
 *   contents will might be modified by method.
 **/
void
cropConcaveHullsByPlaneHulls (const CloudVector &convex_hulls,
    std::vector<CloudVector> &concave_hulls);

/**
 * Convenience method to initialize output vectors, used in method
 * 'checkConcaveHullsNearestNeighbors ()' to make code more readable.
 * Method clears old contents of output vectors and allocates enough
 * memory for subsequent operations.
 *
 * @param[in] concave_cluster_hulls Vector contaning the concave hulls of
 *    object clusters, determines the size of the output vectors
 * @param[out] correspondences Vector of point clouds to hold points of
 *    cluster outlines with correspondences
 * @param[out] no_correspondences Vector of point clouds to hold points of
 *    cluster outlines without correspondences
 * @param[out] scene_neighbors Vector of point clouds to hold the nearest
 *    neighbors on planar surfaces of the points with correspondences
 * @param[out] complete_hulls Vector of point clouds to hold the cluster
 *    outlines projected on all planar surfaces
 **/
void
initializeOutputVectors (const std::vector<CloudVector> &concave_cluster_hulls,
    CloudVector &correspondences,
    CloudVector &no_correspondences,
    CloudVector &scene_neighbors,
    CloudVector &complete_hulls);

/**
 * Method to check if the projected cluster hulls fit with the recored data
 * of the underlying planes. If a hole in any of the planes is caused
 * naturally by occlusion of a regular object, then the projected outline
 * of this object cluster should more or less correspond with the outline
 * of the hole in the planar surface, thus there should be plane inliers
 * near the vertices of the projected outline. This method performs such
 * a neighbor check for a given collection of planar surfaces (input
 * argument 'planes') and several outlines of object clusters, provided
 * by input argument 'concave_clusters'. This input argument needs to hold
 * each cluster outline projected onto each of the planar surfaces. A
 * fitting input vector can be generated via method 'createConcaveHulls()'.
 * This method returns the points of the
 * outlines that have correspondences via output argument
 * 'correspondences' and outline points that don't have a neighbour inside
 * a given radius are returned by output argument 'no_correspondences'.
 * The last output argument 'complete_hulls' contains the cluster outlines
 * projected on all planar surfaces. This will usually only provide
 * meaningful results if the cluster outlines, priovided by
 * 'concave_clusters' have been cropped first via method
 * 'cropConcaveHullsByPlaneHulls ()'.
 *
 * @param[in] planes A vector containing inliers of a planar surface as
 *    elements
 * @param[in] concave_clusters A vector containing vectors of projected
 *    cluster outlines, first index specifies the underlying plane, second
 *    index specifies the number of the cluster
 * @param[out] correspondences  Vector of point clouds holding the points
 *    for each cluster outline that had a correspondence on the planar
 *    surface
 * @param[out] no_correspondences Vector of point clouds containing the
 *    points of an outline that don't have any corresponding point on a
 *    planar surface
 * @param[out] scene_neighbors Vector of point clouds holding the points
 *    of the planar surfaces for each cluster that were closest to a point
 *    with a correspondence (indices match with 'correspondences').
 * @param[out] complete_hulls Vector containing the projection of the
 *    cluster points on all planar surfaces
 **/
void
checkConcaveHullsNearestNeighbors (const CloudVector &planes,
    const std::vector<CloudVector> &concave_clusters,
    CloudVector &correspondences,
    CloudVector &no_correspondences,
    CloudVector &scene_neighbors,
    CloudVector &complete_hulls);

/**
 * Function to check if two bounding boxes of planar patches intersect.
 * It is assumed here, that both bounding boxes are axis aligned, thus
 * greatly simplyfing the general case. To determine if the bounding boxes
 * intersect they are projected into a 2D plane (simply by ignoring the
 * third coordinate). Usually this will be the x-y-plane, however if the
 * projected patches are (almost) parallel to either the x-z-plane or the
 * y-z-plane, the projection into these planes is used and checked for
 * intersections.
 * Note that input arguments are four dimensional vector (for
 * compatibility reasons), but only the first three entries of each input
 * argument are considered.
 *
 * @param[in] box1_min The minimum of the first bounding box
 * @param[in] box1_max The maximum of the first bounding box
 * @param[in] box2_min The minimum of the second bounding box
 * @param[in] box2_max The maximum of the second bounding box
 * @return true if the bounding boxes intersect; false otherwise
 **/
bool
intersectBoundingBoxes (const Eigen::Vector4f &box1_min,
    const Eigen::Vector4f &box1_max,
    const Eigen::Vector4f &box2_min,
    const Eigen::Vector4f &box2_max);

/**
 * Method to remove overlapping points between different projected
 * object clusters. Overlapping points between two clusters, i.e. vertices
 * of on projected outline that are (too) close to vertices of a different
 * projected outline, are removed by this method.
 *
 * @param[in] projected_clusters A vector containing vectors of point
 *    clouds with cluster outline projected on planar surfaces
 * @param[in,out] concave_hulls A vector containing vectors of point
 *    clouds with the concave hulls of the projected clusters, overlapping
 *    points are removed during the method
 **/
void
removeOverlapBetweenConcaveHulls (std::vector<CloudVector> &projected_clusters,
    std::vector<CloudVector> &concave_hulls);

/**
 * Convenience method to color all points of a given point cloud in a
 * specified color.
 *
 * @param[in,out] cloud The point cloud whose points will be colored
 *    uniformly in (r,g,b)
 * @param[in] r Red color channel of the new point color
 * @param[in] g Green color channel of the new point color
 * @param[in] b Blue color channel of the new point color
 **/
void
colorPointCloud (CloudPtr cloud, uint8_t r, uint8_t g, uint8_t b);

/**
 * Convenience method to color all points of a given vector of point
 * clouds in a specified color.
 *
 * @param[in,out] cloud The vector of point clouds whose points will be
 *    colored uniformly in (r,g,b)
 * @param[in] r Red color channel of the new point color
 * @param[in] g Green color channel of the new point color
 * @param[in] b Blue color channel of the new point color
 **/
void
colorPointCloudVector (CloudVector &cloud_vector, uint8_t r, uint8_t g,
    int8_t b);


bool
pointInPolygon2D (const std::vector<Eigen::Vector2i> &polygon, const Eigen::Vector2i &query_point);

void
createSampleRays (const LabelCloud::ConstPtr &base_cloud, LabelCloudPtr &ray_cloud,
//    float sample_dist = STD_SAMPLE_DIST,
    float sample_dist = 0.005f,
    Eigen::Vector3f origin = Eigen::Vector3f::Zero ());

template <class T, class U> T convert(const U&);

template <class T, class U> void insert_coords (const T&, U&);

/**
 * @brief: Templated function to compute the perspective projection of a point
 * onto a plane.
 * Perspective projection is done by computing the intersection of the line
 * defined by the point and the origin with the given plane. If the intersection
 * exists the function returns true and the intersection is returned via the
 * output argument. Otherwise the function will return false.
 * Note that the output argument  will retain all point attributes of the input
 * point, apart from its position, if the point cloud be projected into the plane.
 * If no projection exists and the function returns false, the output argument is
 * undefined.
 * @param[in] point The point that is to be projected
 * @param[out] projected_point The projected point, if it exists, otherwise a
 *  copy of 'point'
 * @param[in] plane The plane coefficients
 * @param[in] angle_eps Threshold to determine if the line defined by the origin
 *  and 'point' is parallel to the given plane
 * @returns true if the point could be projected onto the plane, false otherwise
 */
template <typename PointT> inline bool
projectPointOnPlane (const PointT &point, PointT &projected_point, const Eigen::Vector4f &plane,
    double angle_eps = LINE_PLANE_ANGLE_EPS)
{
  // copy point attributes into output
  Eigen::Vector3f origin = Eigen::Vector3f::Zero ();
  Eigen::Vector3f query_point (point.x, point.y, point.z);
  Eigen::Vector4f result_point;
  // check if the line defined by 'point' and the origin intersects the plane
  bool result = lineWithPlaneIntersection (origin, query_point, plane, result_point, angle_eps);
  // copy original point attributes
  projected_point = point;
  // overwrite location
  projected_point.x = result_point[0];
  projected_point.y = result_point[1];
  projected_point.z = result_point[2];
  return result;
}

/**
  * @brief: Method to project a complete onto a given plane.
  * Points are projected perspectively, assuming the observer at the origin,
  * i.e., at (0,0,0). Uses 'projectPointOnPlane ()' to project the individual
  * points.
  * @param[in] input The point cloud that is projected onto the given plane.
  * @param[in] plane The plane coefficients as an Eigen::Vector4f.
  * @param[out] output The projected point cloud.
  * @param[in] angle_eps Threshold to determine if a point can't be projected
  *   into the plane, because the vector from origin to point doesn't intersect
  *   the plane.
  */
template <typename PointT> inline void
projectPointCloudOnPlane (const typename pcl::PointCloud<PointT>::Ptr &input,
    const Eigen::Vector4f &plane, typename pcl::PointCloud<PointT>::Ptr &output,
    double angle_eps = LINE_PLANE_ANGLE_EPS)
{
  output->points.clear ();
  output->points.reserve (input->points.size ());

  PointT projected_point;
  typename pcl::PointCloud<PointT>::VectorType::const_iterator p_it;
  p_it = input->points.begin ();
  while (p_it != input->points.end ())
  {
    if (projectPointOnPlane<PointT> (*p_it++, projected_point, plane, angle_eps))
    {
      output->points.push_back (projected_point);
    }
  }
  output->width = output->points.size ();
  output->height = 1;
  output->header = input->header;
}

/**
* @brief: Method to convert a HSV color value to a RGB triple.
* It is assumed that saturation and value are both 1.0f, so only bright
* and full colors will be returned.
* @param[in] h The hue angle in degrees
* @param[out] r The red value as a float in [0.0, 1.0]
* @param[out] g The green value as a float in [0.0, 1.0]
* @param[out] b The blue value as a float in [0.0, 1.0]
*/
void
hsv2rgb (float h, float &r, float &g, float &b);


/* @brief: Template method to create a triangulated mesh of a given 2D convex hull.
 * The method expects the convex hull in terms of a point cloud and will return
 * a ros visualization_msgs::Marker containing a triangle list that describes
 * area enclodes by the convex hull. The points in the convex hull need to be in
 * order for a proper tesselation.
 *
 * @param[in] hull_cloud The convex hull as a point cloud.
 * @param[out] marker A visualization_msgs::Marker describing the enclosed area
 *  as a triangle list
 */
template <typename PointT> inline bool
tesselateConvexHull (const typename pcl::PointCloud<PointT>::Ptr &hull_cloud,
    visualization_msgs::Marker &marker)
{
  if (hull_cloud->points.size () < 3)
  {
    ROS_WARN ("Retrieved 'convex hull' consisting of only %lu points, ignoring",
        hull_cloud->points.size ());
    return false;
  }

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.points.clear ();
  marker.points.reserve ((hull_cloud->points.size () - 2) * 3);
  geometry_msgs::Point fixed_point, tp1, tp2;

  // pick first point of convex hull polygon to belong to each tesselation triangle
  typename pcl::PointCloud<PointT>::VectorType::const_iterator p_it = hull_cloud->points.begin ();
  fixed_point.x = p_it->x;
  fixed_point.y = p_it->y;
  fixed_point.z = p_it->z;
  p_it++;
  tp1.x = p_it->x;
  tp1.y = p_it->y;
  tp1.z = p_it->z;
  p_it++;
  while (p_it != hull_cloud->points.end ())
  {
    tp2.x = p_it->x;
    tp2.y = p_it->y;
    tp2.z = p_it->z;

    marker.points.push_back (fixed_point);
    marker.points.push_back (tp1);
    marker.points.push_back (tp2);
    tp1 = tp2;
    p_it++;
  }
  return true;
}

/* @brief: Template method to create a triangulated mesh of the occlusion frustum for
 * a given convex hull.
 * The method expects the convex hull in terms of a point cloud and will return
 * a ros visualization_msgs::Marker containing a triangle list that describes
 * area enclodes by the convex hull. The final argument provides the origin, i.e.,
 * the resultant triangle list will consist of triangles formed by the line segments
 * of the convex hull as baselines and the origin at the top. If no origin is
 * provided (0,0,0) will be assumed as origin.
 *
 * @param[in] hull_cloud The convex hull as a point cloud.
 * @param[out] marker A visualization_msgs::Marker describing the enclosed area
 *  as a triangle list
 * @param[in,optional] origin The original viewpoint
 */
template <typename PointT> inline bool
tesselateConeOfHull (const typename pcl::PointCloud<PointT>::Ptr &hull_cloud,
    visualization_msgs::Marker &marker, geometry_msgs::Point *origin = NULL)
{
  if (hull_cloud->points.size () < 2)
  {
    ROS_WARN ("Retrieved 'convex hull' consisting of only %lu points, ignoring",
        hull_cloud->points.size ());
    return false;
  }

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.points.clear ();
  marker.points.reserve ((hull_cloud->points.size ()) * 3);
  geometry_msgs::Point fixed_point, tp1, tp2;

  // pick the origin (= viewpoint) as fixed point
  typename pcl::PointCloud<PointT>::VectorType::const_iterator p_it = hull_cloud->points.begin ();
  if (origin == NULL)
  {
    fixed_point.x = 0.0f;
    fixed_point.y = 0.0f;
    fixed_point.z = 0.0f;
  }
  else
  {
    fixed_point.x = origin->x;
    fixed_point.y = origin->y;
    fixed_point.z = origin->z;
  }
  tp1.x = p_it->x;
  tp1.y = p_it->y;
  tp1.z = p_it->z;
  p_it++;
  while (p_it != hull_cloud->points.end ())
  {
    tp2.x = p_it->x;
    tp2.y = p_it->y;
    tp2.z = p_it->z;

    marker.points.push_back (fixed_point);
    marker.points.push_back (tp1);
    marker.points.push_back (tp2);
    tp1 = tp2;
    p_it++;
  }
  marker.points.push_back (fixed_point);
  marker.points.push_back (tp1);
  tp2.x = hull_cloud->points.front ().x;
  tp2.y = hull_cloud->points.front ().y;
  tp2.z = hull_cloud->points.front ().z;
  marker.points.push_back (tp2);

  return true;
}

/**
  * @brief Template function to return the distance of a point to a given line.
  * The line is defined as passing through the points provided by the first two arguments
  * while its distance to the point provided by the third argument will be returned.
  *
  * @param[in] line_point_a The first point on the line
  * @param[in] line_point_b The second point on the line
  * @param[in] query_point The query point
  * @returns The distance from the query point to the line
  */
template <typename PointT> inline float
lineToPointDistance (const PointT &line_point_a, const PointT &line_point_b, const PointT &query_point)
{
  Eigen::Vector3f a (line_point_a.x, line_point_a.y, line_point_a.z);
  Eigen::Vector3f b (line_point_b.x, line_point_b.y, line_point_b.z);
  Eigen::Vector3f c (query_point.x, query_point.y, query_point.z);
  Eigen::ParametrizedLine<float, 3> param_line = Eigen::ParametrizedLine<float, 3>::Through (a, b);
  return param_line.distance (c);
}

/**
  * @brief Non-template function to return the distance of a point to a given line segment.
  * The line segment its defined by its start and end point, provided in the first two
  * arguments.
  *
  * @param[in] segment_start The start point of the line segment.
  * @param[in] segment_end The end point of the line segment.
  * @param[in] query_point The query point of which the distance to the segment is desired.
  * @returns The distance from the query point to the line segment.
  */
float
lineSegmentToPointDistance (const Eigen::Vector3f &segment_start, const Eigen::Vector3f &segment_end,
    const Eigen::Vector3f query_point);

/**
  * @brief Template function to return the distance of a point to a given line segment.
  * The line segment its defined by its start and end point, provided in the first two
  * arguments. Now just converts the templated arguments to Eigen::Vector3f and calls
  * the non-templated version of the function.
  *
  * @param[in] segment_start The start point of the line segment.
  * @param[in] segment_end The end point of the line segment.
  * @param[in] query_point The query point of which the distance to the segment is desired.
  * @returns The distance from the query point to the line segment.
  */
template <typename PointT> inline float
lineSegmentToPointDistance (const PointT &segment_start, const PointT &segment_end, const PointT &query_point)
{
  Eigen::Vector3f a (segment_start.x, segment_start.y, segment_start.z);
  Eigen::Vector3f b (segment_end.x, segment_end.y, segment_end.z);
  Eigen::Vector3f c (query_point.x, query_point.y, query_point.z);

  return lineSegmentToPointDistance (a, b, c);
}

/**
 * @brief Template method to convert some pcl point into a geometry_msgs::Point.
 *
 * @param[in] point The pcl point
 * @param[out] out_point The corresponding geometry_msgs::Point
 */
template <typename PointT> inline void
convert (const PointT &point, geometry_msgs::Point &out_point)
{
  out_point.x = point.x;
  out_point.y = point.y;
  out_point.z = point.z;
}

/**
 * @brief Template function to return a mesh represenatation of a given 3D convex hull.
 * The 3D hull needs to be given as a point cloud and a representation (via indices) of
 * its facets. A message marker containing a mesh (with correctly oriented facets) and the
 * center of gravity of the convex hull are returned via output arguments.
 *
 * @param[in] hull_cloud The point cloud representing the 3D convex hull
 * @param[in] facets The factes of the convex hull as a vector of pcl::Vertices
 * @param[out] maker The visualization_msgs::Marler containing the mesh representation
 * @param[out] center_of_gravity The center point of the convex hull
 * @param[out] mesh A representation of the convex hull as a shape_msgs::Mesh
 */
template <typename PointT> inline void
tesselate3DConvexHull (const typename pcl::PointCloud<PointT>::Ptr &hull_cloud,
    const std::vector<pcl::Vertices> &facets, visualization_msgs::Marker &marker,
    Eigen::Vector3f &center_of_gravity, shape_msgs::Mesh &mesh)
{
  // clear and prepare output argument
  mesh.triangles.clear ();
  mesh.vertices.clear ();
  mesh.triangles.reserve (facets.size ());
  mesh.vertices.reserve (hull_cloud->points.size ());

  std::vector<Eigen::Vector3f> vertices;
  vertices.reserve (hull_cloud->points.size ());
  center_of_gravity = Eigen::Vector3f::Zero ();
  geometry_msgs::Point tmp_point;
  Eigen::Vector3f normal, v1, v2, v3;
  // compute center of gravity of convex hull and convert all points
  typename pcl::PointCloud<PointT>::VectorType::const_iterator p_it = hull_cloud->points.begin ();
  while (p_it != hull_cloud->points.end ())
  {
    vertices.push_back (convert<Eigen::Vector3f, PointT> (*p_it));
    convert<PointT> (*p_it, tmp_point);
    mesh.vertices.push_back (tmp_point);

    center_of_gravity[0] += p_it->x;
    center_of_gravity[1] += p_it->y;
    center_of_gravity[2] += p_it->z;
    p_it++;
  }
  center_of_gravity /= static_cast<float> (hull_cloud->points.size ());

  marker.points.clear ();
  marker.points.reserve (facets.size () * 3);
  shape_msgs::MeshTriangle current_face;
  for (size_t i = 0; i < facets.size (); ++i)
  {
    if (facets[i].vertices.size () != 3)
    {
      ROS_WARN ("received facet for convex hull with %lu != 3 vertices; ignoring!", facets[i].vertices.size ());
      continue;
    }
    // check (and correct if neccessary) face orientation
    v1 = vertices[facets[i].vertices[0]];
    v2 = vertices[facets[i].vertices[1]];
    v3 = vertices[facets[i].vertices[2]];
    normal = (v3 - v1).cross (v2 - v1);
    marker.points.push_back (mesh.vertices[facets[i].vertices[0]]);
    current_face.vertex_indices[0] = facets[i].vertices[0];
    if ((v1 - center_of_gravity).dot (normal) > 0.0f)
    {
      marker.points.push_back (mesh.vertices[facets[i].vertices[2]]);
      marker.points.push_back (mesh.vertices[facets[i].vertices[1]]);
      current_face.vertex_indices[1] = facets[i].vertices[2];
      current_face.vertex_indices[2] = facets[i].vertices[1];
    }
    else
    {
      marker.points.push_back (mesh.vertices[facets[i].vertices[1]]);
      marker.points.push_back (mesh.vertices[facets[i].vertices[2]]);
      current_face.vertex_indices[1] = facets[i].vertices[1];
      current_face.vertex_indices[2] = facets[i].vertices[2];
    }
    mesh.triangles.push_back (current_face);
  }
}


/**
  * @brief Function to compute the minimal distance between two 2D convex hull polygons.
  *
  * Computes the minimal distance between 2 convex hulls given as vectors of type
  * 'Eigen::Vector3f'. It is assumed that the entries in the vectors are ordered properly,
  * i.e., that there is a line segment between consecutive point and between the first and
  * the last point as well.
  * While it is assumed that the convex hulls are 2D, they can be arbitrarily oriented in 3D
  * space.
  * Note: The function assumes that the convex hulls do not overlap and does in fact not
  * test for this. If the hulls could overlap (and thus the distance between them should
  * be 0) a initial check if any point of convex_hull_a lies inside convex_hull_b (or vice
  * versa) should be performed first to test for overlapping. In general this function will
  * not return the correct distance, i.e., 0 for overlapping convex hulls.
  *
  * @param[in] convex_hull_a The polygon describing the first convex hull polygon.
  * @param[in] convex_hull_b The polygon describing the second convex hull polygon.
  * @returns The minimal distance between convex_hull_a and convex_hull_b.
  */
float
convexHullsMinDistance (const std::vector<Eigen::Vector3f> &convex_hull_a,
    std::vector<Eigen::Vector3f> &convex_hull_b);

template <typename PointT> inline float
convexHullsMinDistance (const typename pcl::PointCloud<PointT>::ConstPtr &convex_hull_a,
   const typename pcl::PointCloud<PointT>::ConstPtr &convex_hull_b)
{
  std::vector<Eigen::Vector3f> c_hull_a;
  std::vector<Eigen::Vector3f> c_hull_b;

  convert<PointT> (convex_hull_a, c_hull_a);
  convert<PointT> (convex_hull_b, c_hull_b);

  return convexHullsMinDistance (c_hull_a, c_hull_b);
}

/**
  * @brief Function to check if the distance between two non-intersection 2D convex hulls
  * is below the given threshold.
  * Checks if the distance between 2 convex hulls given as vectors of type 'Eigen::Vector3f'
  * is below the specified threshold. It is assumed that the entries in the vectors are
  * ordered properly, i.e., that there is a line segment between consecutive point and
  * between the first and the last point as well.
  * While it is assumed that the convex hulls are 2D, they can be arbitrarily oriented in 3D
  * space.
  * Note: Threshold must be positive (unchecked) and the convex hulls should not overlap /
  * intersect. Otherwise the behaviour is undefined.
  *
  * @param[in] convex_hull_a The polygon describing the first convex hull polygon.
  * @param[in] convex_hull_b The polygon describing the second convex hull polygon.
  * @param[in] threshold The specified distance threshold.
  * @returns True if the distance is below the given threshold, false otherwise.
  */

bool
convexHullDistBelowThreshold (const std::vector<Eigen::Vector3f> &convex_hull_a,
    std::vector<Eigen::Vector3f> &convex_hull_b, float threshold);

template <typename PointT> inline bool
convexHullDistBelowThreshold (const typename pcl::PointCloud<PointT>::ConstPtr &convex_hull_a,
   const typename pcl::PointCloud<PointT>::ConstPtr &convex_hull_b, float threshold)
{
  std::vector<Eigen::Vector3f> c_hull_a;
  std::vector<Eigen::Vector3f> c_hull_b;

  convert<PointT> (convex_hull_a, c_hull_a);
  convert<PointT> (convex_hull_b, c_hull_b);

  return convexHullDistBelowThreshold (c_hull_a, c_hull_b, threshold);
}


/**
  * @brief: Convenience function to receive the center of gravity or centroid
  * of a given vector of 'Eigen::vector3f'.
  *
  * @param[in] vec The vector of which the centroid is returned.
  * @returns The centroid of the input vector
  */
Eigen::Vector3f getCentroid (const std::vector<Eigen::Vector3f> &vec);

/**
  * @brief: method to filter a vector of points, according to their position to a given
  * hyperplane.
  * All points in the vector, given by input argument 'all_points' are tested if they lie
  * on the positive side of the given hyperplane (i.e., if the signed distance is larger than 0).
  * All points for which this holds true are inserted into and returned via output argument
  * 'points_on_pos_side'.
  *
  * @param[in] all_points The points that will be filtered
  * @para,[in] hyperplane The hyperplane that is used as a filter criteria.
  * @param[out] points_on_pos_side The points from 'all_points' that are on the positive
  *   side of the hyperplane.
  */
void getPointsOnPositiveSideOfHyperplane (const std::vector<Eigen::Vector3f> &all_points,
    const Eigen::Hyperplane<float, 3> &hyperplane, std::vector<Eigen::Vector3f> &points_on_pos_side);

/**
  * @brief: Function to check if two 2D convex hulls intersect, i.e., if at least one point of
  * the first convex hull lies inside the second or vice versa.
  *
  * The convex hulls need to be 2 dimensional (thus they form closed polygons) and be aligned in
  * same hyperplane in 3D space.
  * Both convex hulls need to be given as std::vector<Eigen::Vector3f> with the ordering
  * corresponding to the arrangement of the vertices of the convex hull polygons, i.e., between
  * subsequent entries in the vector an edge is assumed (and between the first and the last
  * element).
  * Note that if the polygons are not aligned in the same plane, are not 2 dimensional or not
  * convex the behaviour is undefined.
  *
  * @param[in] convex_hull_a The polygon describing the first convex hull polygon.
  * @param[in] convex_hull_b The polygon describing the second convex hull polygon.
  * @returns True if the convex hulls intersect, false otherwise
  */
bool
doConvexHulls2DIntersect (const std::vector<Eigen::Vector3f> &convex_hull_a,
    const std::vector<Eigen::Vector3f> &convex_hull_b);

/**
  * @brief: Templated version of 'doConvexHulls2DIntersect ()' where convex hulls are represented
  * as point clouds rather, than vectors of 'Eigen::Vector3f'.
  * Internally converts the point clouds and uses the non-template version of
  * 'doConvexHulls2DIntersect ()' (refer there for additional documentation).
  *
  * @param[in] convex_hull_a The polygon describing the first convex hull polygon.
  * @param[in] convex_hull_b The polygon describing the second convex hull polygon.
  */
template <typename PointT> inline bool
doConvexHulls2DIntersect (const typename pcl::PointCloud<PointT>::ConstPtr &convex_hull_a,
   const typename pcl::PointCloud<PointT>::ConstPtr &convex_hull_b)
{
  std::vector<Eigen::Vector3f> c_hull_a;
  std::vector<Eigen::Vector3f> c_hull_b;

  convert<PointT> (convex_hull_a, c_hull_a);
  convert<PointT> (convex_hull_b, c_hull_b);

  return doConvexHulls2DIntersect (c_hull_a, c_hull_b);
}

/**
  * @brief: Method to retrieve the axis aligned bounding box of a given
  * collection 2D coordinates.
  *
  * @param[in] border The collection of 2D coordinates
  * @param[out] min_bbox The minimal corner of the 2D coordinate border
  * @param[out] max_bbox The maximal corner of the 2D coordinate border
  */
void
getBBox (const std::vector<Eigen::Vector2i> &border, Eigen::Vector2i &min_bbox, Eigen::Vector2i &max_bbox);


/**
  * @brief: small helper method, used by storeCloudAsColorImage ().
  */
void
storeAsImage (const std::vector<std::vector<Eigen::Vector3i> > &img_data, const std::string &filename);

/**
  * @brief: Templated method to store a given organized pointcloud as an easily
  * debug-able image.
  *
  * Stores a given point cloud into a 'ppm' image. 'ppm' images are easily
  * readable, since they contain three integer values for each pixel. If the
  * provided point cloud doesn't contain color information, the output image
  * is undefined.
  *
  * @param[in] cloud The cloud that will be stored as a color image
  * @param[in] base_filename The base filename (without the file extension)
  */
template <typename PointT> inline void
storeCloudAsColorImage (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const std::string &base_filename)
{
  if (!cloud->isOrganized ())
  {
    std::cerr << "storeCloudAsColorImage(): given cloud not organized, aborting!" << std::endl;
    return;
  }
  CloudPtr color_cloud (new Cloud);
  pcl::copyPointCloud (*cloud, *color_cloud);

  std::vector<std::vector<Eigen::Vector3i> > color_data (color_cloud->width,
      std::vector<Eigen::Vector3i> (color_cloud->height, Eigen::Vector3i::Zero ()));
  ColorPoint tmp_point;

  for (size_t u = 0; u < color_cloud->width; ++u)
  {
    for (size_t v = 0; v < color_cloud->height; ++v)
    {
      tmp_point = color_cloud->at (u, v);
      color_data[u][v][0] = tmp_point.r;
      color_data[u][v][1] = tmp_point.g;
      color_data[u][v][2] = tmp_point.b;
    }
  }

  std::string filename = base_filename + ".ppm";
  storeAsImage (color_data, filename);
}

/**
  * @brief: Templated method to store a given organized pointcloud as an easily
  * debug-able depth image
  *
  * Stores a given point cloud into a 'pgm' image. 'pgm' images are easily
  * readable, since they contain one integer value for each pixel.
  * In a first pass the maximal distance of a measurement point is established.
  * This point will obtain a value of 255 in the resulting depth image (i.e.
  * white), while all other points will be assigned a value in relation to the
  * maximal distance (between 0 and 255). NaN-measurements will be associated
  * with a value of 0.
  *
  * @param[in] cloud The cloud that will be stored as a color image
  * @param[in] base_filename The base filename (without the file extension)
  */
template <typename PointT> inline void
storeCloudAsDepthImage (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const std::string &base_filename)
{
  if (!cloud->isOrganized ())
  {
    std::cerr << "storeCloudAsDepthImage (): given cloud not organized, aborting!" << std::endl;
    return;
  }

  std::vector<std::vector<int> > data (cloud->width, std::vector<int> (cloud->height, 0));
  std::vector<std::vector<float> > dists (cloud->width, std::vector<float> (cloud->height, 0.0f));
  float max_dist = 0.0f;
  float curr_dist;
  PointT tmp;

  for (size_t u = 0; u < cloud->width; ++u)
  {
    for (size_t v = 0; v < cloud->height; ++v)
    {
      tmp = cloud->at (u, v);
      if (pcl::isFinite (tmp))
      {
        curr_dist = sqrt ((tmp.x * tmp.x) + (tmp.y * tmp.y) + (tmp.z * tmp.z));
        dists[u][v] = curr_dist;
        if (curr_dist > max_dist)
        {
          max_dist = curr_dist;
        }
      }
      else
      {
        curr_dist = 0.0f;
      }
    }
  }

  for (size_t u = 0; u < dists.size (); ++u)
  {
    for (size_t v = 0; v < dists[0].size (); ++v)
    {
      data[u][v] = static_cast<int>(dists[u][v] / max_dist * 255.0);
    }
  }

  std::string filename = base_filename + ".pgm";
  std::ofstream img_file (filename.c_str ());
  img_file << "P2\n" << "# point cloud interpreted as depth image\n" << data.size () << " "
        << data[0].size () << "\n255" << std::endl;

  for (size_t v = 0; v < data[0].size (); ++v)
  {
    for (size_t u = 0; u < data.size (); ++u)
    {
      img_file << " " << data[u][v];
    }
    img_file << " " << std::endl;
  }
  img_file.flush ();
  img_file.close ();
}

template <typename PointT> inline void
getOctreeIndices (const Eigen::Vector3d &min_bbox, const PointT &lp, float octree_res,
    int &id_x, int &id_y, int &id_z)
{
  id_x = static_cast<int> ((lp.x - min_bbox[0]) / octree_res);
  id_y = static_cast<int> ((lp.y - min_bbox[1]) / octree_res);
  id_z = static_cast<int> ((lp.z - min_bbox[2]) / octree_res);
}

#endif // TRANSP_OBJ_RECON_TOOLS
