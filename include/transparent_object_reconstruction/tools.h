#ifndef TRANSP_OBJ_RECON_TOOLS
#define TRANSP_OBJ_RECON_TOOLS

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
#include <transparent_object_reconstruction/hole_region.h>
#include <transparent_object_reconstruction/occupancy_grid.h>

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
const double MAX_CORR_DIST = 0.02; // 2cm
const double MAX_SQRD_CORR_DIST = MAX_CORR_DIST * MAX_CORR_DIST;
const float MIN_CORRELATION_RATIO = 0.5f;

// threshold to decide when a grid cell is on a plane
const float ON_PLANE_THRESHOLD = 0.02f;
// threshold for minimal size of connected grid cells for a HoleRegion
const size_t MIN_HOLE_REGION_SIZE = 10;
// threshold to remove hole regions with too much artificial convex
// border cells
float MAX_CONVEX_BORDER_THRESHOLD = 0.33f;

// threshold to determine if a hole is caused by a transparent object
const float MIN_NON_INTERSECED_RATIO = 0.5f;

// threshold for point removal from concave hulls due to potential
// overlap with other hulls
const float MAX_CONC_HULL_CORR_DIST = 0.02f;

const float CONCAVE_HULL_ALPHA = 0.005f;
// TODO: end of blindly importet constants


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
    pcl::VoxelGrid<PointType> &v_grid,
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
 * @param[in] polygon The polygon given as the stored vertices in a point
 *   cloud.
 * @param[in] query_point The query point that will be checked.
 * @return true if the query point lies inside the polygon;
 *   false otherwise
 **/
bool
pointInsidePolygon (Cloud::ConstPtr polygon, PointType query_point);

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
  * convex hull is performed by 'pointInsidePolygon ()'.
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
 * Method transform a HoleRegion back into the scene. Since HoleRegions are
 * established (grown) in an 2D occupancy grid in the x-y-plane they need
 * to be projected back into their appropriate 3D pose to further utilize
 * them in the occlusion analysis. In 3D the HoleRegion is generally
 * presented by the back transformed centers of the occupancy grid. The
 * transformation needed by this method is the original transformation used
 * to transform the plane (containing the HoleRegion) into the x-y-plane.
 * The method computes the inverse for the back transformation.
 *
 * @param[in] hole_region The HoleRegion that will be transformed back into
 *    the scene
 * @param[in] occupancy_grid The occupancy grid, containing information
 *    about the HoleRegion
 * @param[in] plane_transformation The transformation used to tranform the
 *    original plane into the x-y-plane
 * @param[out] trans_hole_cloud The hole region, transformed back into
 *    the scene, represented as a point cloud
 **/
void
transformHoleRegionBackToScene (HoleRegion &hole_region,
    OccupancyGrid &occupancy_grid, Eigen::Affine3f &plane_transformation,
    CloudPtr trans_hole_cloud);

/**
 * Convenience method to transform a vector of HoleRegion back into the
 * scene. Internally uses 'transformHoleRegionBackToScene ()' to
 * transform each HoleRegion passed in the input vector. For details
 * about the working of the transformations refer to the comments
 * of 'transformHoleRegionBackToScene ()'.
 *
 * @param[in] hole_regions A vector containing the HoleRegions to be
 *   transformed back into the scene
 * @param[in] occupancy_grid The grid containing information about the
 *   HoleRegions
 * @param[in] plane_transformation The affine transformation to move the
 *   plane into the x-y-plane
 * @param[out] trans_hole_regions A vector containing each HoleRegion,
 *   transformed back into the scene and represented as a point cloud
 **/
void
transformHoleRegionVectorBackToScene (std::vector<HoleRegion> &hole_regions,
    OccupancyGrid &occupancy_grid,
    Eigen::Affine3f &plane_transformation,
    CloudVector &trans_hole_regions);

/**
 * Method to check which rays from a hole to the camera intersect with
 * some other measurements. This can be used to determine which parts
 * of a hole occured due to occlusion with a measured object and which
 * parts did not. The first argument' hole_cloud' needs to be a point
 * cloud represenation of a HoleRegion, i.e., sampled points inside the
 * hole, detected in a planar surface. The fourth argument 'octree' needs
 * to be an octree containing the remaining measurement points of the
 * scene, while argument 'camera_origin' determines the origin of the
 * camera.
 * Output will be stored in arguments two and three, with the former
 * containing the octree leaves that were intersected by rays from hole to
 * camera and the latter containing all points of 'hole_cloud' where no
 * intersection occured.
 * Note that this method currently might not work correctly for sensors
 * with opening angles above 180°.
 *
 * @param[in] hole_cloud A point cloud representing the hole region
 * @param[out] intersections A point cloud containing intersections of
 *   the rays some measurement points
 * @param[out] remaining_points The points of hole_cloud, whose rays were
 *   not intersected
 * @param[in] octree An octree containing points of the remaining scene
 * @param[in] camera_origin A 3D Vector denoting the origin of the kinect
 * sensor.
 **/
void
getHoleRegionIntersections (CloudPtr hole_cloud, CloudPtr intersections,
    CloudPtr remaining_points,
    pcl::octree::OctreePointCloudSearch<PointType> & octree,
    Eigen::Vector3f &camera_origin);

/**
 * Method to check for intersections between hole-to-camera rays and
 * other measurement points. Essentially the same as the 5 argument
 * version, but assuming that the camera's origin is at (0,0,0).
 *
 * @param[in] hole_cloud A point cloud representing the hole region
 * @param[out] intersections A point cloud containing intersections of
 *   the rays some measurement points
 * @param[out] remaining_points The points of hole_cloud, whose rays were
 *   not intersected
 * @param[in] octree An octree containing points of the remaining scene
 **/
void
getHoleRegionIntersections (CloudPtr hole_cloud, CloudPtr intersections,
    CloudPtr remaining_points,
    pcl::octree::OctreePointCloudSearch<PointType> &octree);

/**
 * Method to check for intersections for a vector of hole regions.
 * Basically just a wrapper, consecutively calling
 * 'getHoleRegionIntersections ()' for each element of the input argument
 * 'hole_clouds'. Accordingly output arguments are also given as vectors
 * of point clouds rather than single point clouds.
 *
 * @param[in] hole_cloud A vector of point clouds representing the hole
 *    regions
 * @param[out] intersections A vector of point clouds containing
 *    intersections of the rays some measurement points
 * @param[out] remaining_points The vector of point cloud, containing the
 *    points for each hole_cloud, whose rays were not intersected
 * @param[in] octree An octree containing points of the remaining scene
 * @param[in] camera_origin A 3D Vector denoting the origin of the kinect
 * sensor.
 **/
void
getHoleRegionIntersectionsVector (CloudVector &hole_clouds,
    CloudVector &intersected_clouds,
    CloudVector &non_intersected_clouds,
    pcl::octree::OctreePointCloudSearch<PointType> &octree,
    Eigen::Vector3f &camera_origin);

/**
 * Method to check for intersections for a vector of hole regions.
 * The same as the 5 argument version, but assuming that the camera
 * origin is at (0,0,0).
 *
 * @param[in] hole_cloud A vector of point clouds representing the hole
 *    regions
 * @param[out] intersections A vector of point clouds containing
 *    intersections of the rays some measurement points
 * @param[out] remaining_points The vector of point cloud, containing the
 *    points for each hole_cloud, whose rays were not intersected
 * @param[in] octree An octree containing points of the remaining scene
 **/
void
getHoleRegionIntersectionsVector (CloudVector &hole_clouds,
    CloudVector &intersected_clouds,
    CloudVector &non_intersected_clouds,
    pcl::octree::OctreePointCloudSearch<PointType> &octree);

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
 * Method to create a point sampling of the border of a hull polygon.
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
void
createHullLines (CloudPtr hull, CloudPtr hull_lines, float step_length);

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
    std::vector<pcl::VoxelGrid<PointType> > &v_grid_vector,
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
 * Method to create several occupancy grids from given voxel grids, clouds
 * and plane equations. The transformed plane equations are used to
 * determine where the planar surfaces end and how resulting hole regions
 * are restriced.
 *
 * @param[in] voxel_grids A vector of voxel grids
 * @param[in] clouds A vector of point clouds (corresponding to the grids)
 * @param[in] transformed_plane_equations A vector of vectors holding
 *   tranformed plane equations
 * @param[out] occupancy_grids A vector of occupancy grids
 **/
void
createOccupancyGrids (std::vector<pcl::VoxelGrid<PointType> > &voxel_grids,
    CloudVector &clouds,
    TransPlaneCoeffsVec &transformed_plane_equations,
    std::vector<boost::shared_ptr<OccupancyGrid> > &occupancy_grids);

/**
 * Method to incorporate convex hulls in the configuration of given
 * occupancy grids. Since the occupancy grids are 2D grids (on the
 * x-y-plane) it is necessary that the convex hull were either computed
 * on planar surfaces transformed onto the x-y-plane or transformed
 * afterwards so that the hulls are correctly aligned with the occupancy
 * grids. Note that the hull lines given by 'hull_lines_vec' need to be
 * represented as a point sampling of the line segements (see
 * 'createHullLines()' and 'createHullLinesVec ()').
 *
 * @param[in] hull_lines_vec A vector of sampled convex hull lines.
 * @param[in,out] occupancy_grids A vector of occupancy grids that are
 *   enriched by the information provided by the convex hull lines
 **/
void
addConvHullLinesToOccupancyGrids (CloudVector &hull_lines_vec,
    std::vector<boost::shared_ptr<OccupancyGrid> > &occupancy_grids);

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
 * @param[in.out] cloud The point cloud whose points will be colored
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
 * @param[in.out] cloud The vector of point clouds whose points will be
 *    colored uniformly in (r,g,b)
 * @param[in] r Red color channel of the new point color
 * @param[in] g Green color channel of the new point color
 * @param[in] b Blue color channel of the new point color
 **/
void
colorPointCloudVector (CloudVector &cloud_vector, uint8_t r, uint8_t g,
    int8_t b);

/**
 * Function for visualization of occupancy grid on the command line
 * (primary for debugging purpose) and storage in a ppm file. The ppm
 * filename is auto-generated and depends on input argument 'plane_index'
 * that specifies the planar surface for that the occupancy grid will be
 * visualized. Input argument 'occupancy_grid' is a 2D array containing
 * the values of the occupancy grid, while arguments 'grid_dim_x' and
 * 'grid_dim_y' denote the dimensions of the 2D array. The detected hole
 * regions in the current occupancy grid need to be passed along via input
 * argument 'all_hole_regions'. The function returns the total number of
 * empty grid cells that lie inside the planar surface.
 *
 * @param[in] occupancy_grid A 2D array containing the values of each
 *   grid cell of the occupancy grid
 * @param[in] grid_dim_x The dimension in x direction of the 2D array
 * @param[in] grid_dim_y The dimension in y direction of the 2D array
 * @param[in] all_hole_regions A vector containing all detected hole
 *   regions in the given planar surface / occupancy grid
 * @param[in] plane_index The index of the plane that is represented by
 *   the visualized occupancy grid
 * @return The total number of empty grid cells
 **/
//TODO: adapt to ros gridmap or something similar
unsigned int
printGridToConsole (grid_values** occupancy_grid, unsigned int grid_dim_x,
    unsigned int grid_dim_y, std::vector<HoleRegion> &all_hole_regions,
    size_t plane_index);


bool
pointInPolygon2D (const std::vector<Eigen::Vector2i> &polygon, const Eigen::Vector2i &query_point);

void
createSampleRays (const LabelCloud::ConstPtr &base_cloud, LabelCloudPtr &ray_cloud,
//    float sample_dist = STD_SAMPLE_DIST,
    float sample_dist = 0.005f,
    Eigen::Vector3f origin = Eigen::Vector3f::Zero ());

/**
 * @brief Function to project a point into a given plane via a raytracing approach.
 * For the projection a ray between the given input point and the origin (at
 * (0,0,0) is created and checked for intersection with the plane, specified
 * by the ModelCoefficientsPtr argument. If such an intersection exists it is
 * returned via output argument 'projected_point' and the function returns true.
 *
 * @param[in] input The point that will be projected into the plane
 * @param[out] projected_point The point projected into the plane, if it exists
 * @param[in] plane The model coefficients describing the plane
 * @returns true if an intersection of the created ray and the plane exists;
 * false otherwise
 */
bool
projectPointOnPlane (const PointType &input, PointType &projected_point, const Eigen::Vector4f &plane);

bool
projectPointOnPlane (const LabelPoint &input, LabelPoint &projected_point, const Eigen::Vector4f &plane);

template <class T, class U> T convert(const U&);

template <class T, class U> void insert_coords (const T&, U&);

/**
 * Templated function to compute the intersection between a line and a plane.
 * This function is very similar to the 5 argument function with the
 * same name. However the first point on the line is in this case fixed
 * to be the origin (0,0,0), so that only on other point on the line
 * needs to be specified. If an intersection exists, this is returned via
 * the output argument and the function returns true. Otherwise it will
 * return false and the intersection will be set to 'original_point'.
 *
 * @param[in] original_point Point on the line
 * @param[in] plane The coefficients for the plane
 * @param[out] intersection_point Intersection between line and plane if
 *   it exists.
 * @param[in] angle_eps Threshold to check for parallelity
 * @return true if the line intersects the plane; false otherwise
 **/

template <class T> bool lineWithPlaneIntersection (const T &original_point, const Eigen::Vector4f &plane,
    T &intersection_point, double angle_eps);

#endif // TRANSP_OBJ_RECON_TOOLS
