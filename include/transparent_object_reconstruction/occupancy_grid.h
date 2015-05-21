#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>

#include <transparent_object_reconstruction/common_typedefs.h>
#include <transparent_object_reconstruction/hole_region.h>

typedef std::vector<CloudPtr> CloudVector;

typedef std::vector<Eigen::Vector4f> Vector4fVec;

/**
 * Enumeration of the different possible grid values. These consist of
 * 'grid_empty': An empty grid cell (possible a hole)
 * 'grid_occupied': Some sensor measurements are inside this cell
 * 'grid_border': Since the grid has only finite dimensions, these cells
 *    create a 'frame' arouud the grid (needed to distinguish if holes are
 *    inside the planar surfaces or not)
 * 'grid_plane_border': A grid cell that lies on the intersection with an
 *    intersecting planar surface
 * 'grid_behind_plane': Marks grid cells that are located behind the
 *    intersection of the grid plane with a different planar surface
 * 'grid_hole_cluster': ???
 * 'grid_conv_hull_border': A grid cell that lies on the convex hull of
 *    the underlying planar surface
 **/
enum grid_values
{
  grid_empty,
  grid_occupied,
  grid_border,
  grid_plane_border,
  grid_behind_plane,
  grid_hole_cluster,
  grid_conv_hull_border
};

/**
 * Class to model a 2D occupancy grid, that can be used to extract
 * connected regions without any measurement points. These holes are
 * stored via class 'HoleRegion' and detected by a simple region growing
 * algorithm. The computed HoleRegions are located (mainly) inside the
 * detected planar surface that was used to initialize the OccupancyGrid.
 * The underlying data structure of the OccupancyGrid is given by a
 * 'pcl::VoxelGrid<>' and it is assumed that the point inliers of the
 * planar are first transformed into the x-y-plane before construction of
 * the OccupancyGrid. Furthermore other detected planar surfaces can be
 * specified by their plane coefficients (to which the same transformation
 * was applied that moved the plane inliers into the x-y-plane) and are
 * considered during computation of the HoleRegions (an intersection with
 * a different planar surface is a valid border with respect to the
 * 'interiority' of a hole).
 **/
class
OccupancyGrid
{
  // the underlying voxelgrid
  pcl::VoxelGrid<PointType> voxel_grid;
  // a pointer to the input point cloud containing planar surface inliers
  CloudPtr cloud;
  // denote the dimensions in x and y direction in terms of grid cells
  size_t x_dim, y_dim;
  // 2D array of grid_values to denote the contents of the OccupancyGrid
  grid_values **occupancy_grid;
  // 2D array showing for each cell if it was already visited
  bool **visited;
  // denoting the corners of the bounding box in terms of grid coordinates
  Eigen::Vector3i min_box, max_box;
  // corners of the bounding box in measurement coords and a query point
  Eigen::Vector4f min_point, max_point, test_point;
  // denoting the dimensions of a single grid cell as a Vector3f
  Eigen::Vector3f leaf_size;
  // vector to hold the transformed plane equations of other planes
  std::vector<Vector4fVec> transformed_plane_equations;
  // denoting the index of the input planar surface
  size_t index;

  // distance threshold to determine if a point lies on an intersecting plane
  static float on_plane_threshold;
  // minimal size of a valid hole region in terms of empty connected cells
  static size_t min_hole_region_size;

  public:

  /**
   * Constructor to create a new instance of OccupancyGrid. Needs an
   * already initialized 'pcl::VoxelGrid' and a pointer to the point
   * cloud containing the planar surface inliers that were used to the
   * fill the voxel grid (given by input argument 'cloud').
   * Note that the planar inliers in 'cloud' need to be aligned with the
   * x-y-plane first by a transformation before creating the voxel grid.
   * In addition a vector containing vectors of transformed plane
   * equations of all other detected planar surfaces is needed together
   * with the index, specifying at which position in vector
   * 'transformed_plane_equations' the needed equtions for the intersection
   * with the planar inliers (aligned with the x-y-plane) are contained.
   *
   * @param[in] voxel_grid A pcl::VoxelGrid constructed from the planar
   *    inliers
   * @param[in] cloud The point cloud containing the planar inliers
   * @param[in] transformed_plane_equations The transformed plane equations
   * @param[in] index The index specifying which entry in
   *    'transformed_plane_equations' is needed for plane intersections
   **/
  OccupancyGrid (pcl::VoxelGrid<PointType> &voxel_grid, CloudPtr cloud,
      std::vector<Vector4fVec > &transformed_plane_equations,
      size_t index);

  /**
   * Destructor to free allocated ressources.
   **/
  ~OccupancyGrid (void);

  /**
   * Method to fill the OccupancyGrid. Via the bounding box of the planar
   * inliers the dimensions of the grid are determined and for every grid
   * cell the approriate value is determined via function
   * 'calcGridValue ()'.
   * Note: This should be called before HoleRegions are extracted (via
   * 'createHoleRegions ()'.
   **/
  void
  fillGrid (void);

  /**
   * Method to add the convex hull of the underlying planar inliers to the
   * occupancy grid. The convex hull in this case needs to be represented
   * as a point cloud containing point samples of the hull's line segments
   * and need to have been transformed like to planar inliers into the
   * x-y-plane. If the convex hull line passes through a grid cell that is
   * empty, the value of this cell is changed and marked to form the border
   * of the convex hull. These borders in turn can be used to restrict
   * potential hole regions.
   *
   * @param[in] hull_lines A point cloud containing the sampled outline of
   *    the convex hull of the planar surface inliers
   **/
  void
  addConvHullLines (const CloudPtr hull_lines);

  /**
   * Function to get the OccupancyGrid as a 2D array of 'grid_values'.
   * Along with the array the 2 dimensions (first dimension via 'x_dim'
   * and second dimension via 'y_dim') are returned by the output
   * arguments.
   *
   * @param[out] x_dim The first dimension of the returned
   * @param[out] y_dim The second dimension of the returned
   * @return The OccupancyGrid as a 2D array of 'grid_values'
   **/
  grid_values**
  retrieveOccupancyGrid (size_t &x_dim, size_t &y_dim);

  /**
   * Method to fill a vector with instances of HoleRegion to represent
   * the holes in the planar surface of the current OccupancyGrid.
   *
   * @param[out] hole_regions A vector containing all HoleRegions in the
   *    current OccupancyGrid
   **/
  void
  createHoleRegions (std::vector<HoleRegion> &hole_regions);
  
  /**
   * Function to return the HoleRegion that contains the specified grid
   * coordinates, if it exits and was not already returned previously.
   * If the grid cell at position specified by input arguments 'x_index'
   * and 'y_index' is empty and was not previously visited method
   * 'recursiveGrowHoleRegion ()' is called to create a new HoleRegion
   * with a region growing approach. This new HoleRegion is returned.
   * If the grid cell at the specified position is not empty or if it
   * was already visited a 0-pointer is returned.
   *
   * @param x_index The x-coord in grid coordinates where a new
   *    HoleRegion is created
   * @param y_index The y-coord in grid coordinates where a new
   *    HoleRegion is created
   * @return A new HoleRegion if it exists and was not returned previously
   *    a 0-pointer otherwise
   **/
  HoleRegion*
  createHoleRegion (size_t x_index, size_t y_index);

  /**
   * Method to recursively grow an existing HoleRegion at the position
   * specified by the given grid coordinates. Implements a region growing
   * approach: If the grid cell specified by coordinates 'x_index' and
   * 'y_index' has not been visited, yet and is empty it will be added
   * to the HoleRegion given by argument 'hole_region' and the method
   * is called recursively (hence the name) for its four neighbors (von
   * Neumann neighborhood). If the cell is not empty, it might cause the
   * HoleRegion to marked as adjacent to the grid border (if the cell 
   * belongs to the grid border). Not visited cells are markes as visited
   * by this method and visited cells will not be considered, so that each
   * cell is only used once in the region growing approach.
   *
   * @param[in] x_index The x-coordinate in grid coordinates the might
   *    be added to the HoleRegion
   * @param[in] y_index The y-coordinate in grid coordinates the might
   *    be added to the HoleRegion
   * @param[in, out] hole_region Pointer to the HoleRegion that might grow
   *    via region growing approach
   **/
  void
  recursiveGrowHoleRegion (size_t x_index, size_t y_index,
      HoleRegion* hole_region);

  /**
   * Function to return the appropriate value for a given grid cell.
   * Input arguemtn 'test_point' denotes the center of a grid cell in
   * measurement units (and not in grid cell coordinates), while
   * 'grid_index' specifies the cell index in the underlying
   * pcl::VoxelGrid (-1 for empty cells in the VoxelGrid). Argument
   * 'transformed_plane_equations' holds vectors of transformed plane
   * equations. In each vector all plane equations are transformed
   * according to the transformation the aligns the plane at the current
   * index with the x-y-plane. Lastly argument 'index' specifies the index
   * of the underlying planar surface in 'transformed_plane_equations'.
   *
   * @param[in] test_point A test point denoting the center of a grid cell
   *    in measurement units
   * @param[in] transformed_plane_equations A vector holding vectors of
   *    transformed plane equations in turn
   * @param[in] index The index of the underlying planar surface of the
   *    current OccupancyGrid
   * @param[in] grid_index The index in the underlying pcl::VoxelGrid for
   *    the given 'test_point'
   * @ return The value from enum 'grid_values' to store for the specified
   *    grid cell
   **/
  grid_values
  calcGridValue (const Eigen::Vector4f &test_point,
    const std::vector<Vector4fVec> &transformed_plane_equations,
    size_t index, int grid_index);

  /**
   * Function to retrieve the minimal corner of the enclosing axis aligned
   * bounding box for the underlying planar inliers as a 'PointType'.
   * Note that while 'PointType' may contain information beyond 3D
   * coordinates only the x,y and z values will be set.
   *
   * @return The minimal corner of the bounding box as a 'PointType'
   **/
  PointType
  getMinPoint (void);

  /**
   * Function to retrieve the maximal corner of the enclosing axis aligned
   * bounding box for the underlying planar inliers as a 'PointType'.
   * Note that while 'PointType' may contain information beyond 3D
   * coordinates only the x,y and z values will be set.
   *
   * @return The maximal corner of the bounding box as a 'PointType'
   **/
  PointType
  getMaxPoint (void);

  /**
   * Function to retrieve the grid cell / leaf size of the current
   * OccupancyGrid in measurement units, i.e., the returned Vector3f
   * contains the length of a grid cell in the three dimensions.
   *
   * @return The size of a leaf as an 'Eigen::Vector3f'
   **/
  Eigen::Vector3f
  getLeafSize (void);

  /**
   * Setter method to set the minimal size of a HoleRegion. That means the
   * minimal number of empty connected grid cells that will form a
   * detected hole is specified via this method.
   *
   * @param[in] min_size The minimal number of connected empty grid cells
   *    that is needed to compose a HoleRegion
   **/
  static void
  setMinHoleRegionSize (size_t min_size);
  /**
   * Setter method to set the distance threshold for a grid cell to be
   * considered on an intersecting plane.
   *
   * @param[in] threshold The distance threshold
   **/
  static void
  setOnPlaneThreshold (float threshold);

  /**
   * Getter function to retrive the number of connected empty grid cells
   * to compose a HoleRegion.
   *
   * @return The number of empty grid cells needed for a hole.
   **/
  static size_t
  getMinHoleRegionSize (void);
  /**
   * Getter function to retrieve the distance threshold for a grid cell
   * to be considered lying on an intersecting plane.
   *
   * @return The distance threshold
   **/
  static float
  getOnPlaneThreshold (void);


  /**
   * Method to recursively check the borders of a given HoleRegion.
   * Checks the grid cell specified by grid coordinates 'x_index' and
   * 'y_index': If it belongs to the HoleRegion the method is called
   * recursively for all neighbors of the grid cell otherwise the number
   * of border cells for the given HoleRegion is increased. The grid cell
   * specified via 'x_index' and 'y_index' is marked as checked (works as
   * recursion base).
   *
   * @param[in] x_index First coordinate of the grid cell to be checked
   * @param[in] y_index Second coordinate of the grid cell to be checked
   * @param[in,out] hole_region The HoleRegion for which the number of
   *    border cells is determined
   * @param[in,out] checked 2D bool array to determine if the specified
   *    grid cell was alredy checked
   **/
  void
  recursiveCheckHoleRegionBorder (size_t x_index, size_t y_index,
      HoleRegion &hole_region, bool** checked);

  /**
   * Method to determine the border size (in terms of border cells) for an
   * extracted HoleRegion. Uses 'recursiveCheckHoleRegionBorder ()'.
   *
   * @param[in,out] hole_region The HoleRegion of that the border will be
   *    checked
   **/
  void
  checkHoleRegionBorder (HoleRegion &hole_region);

  /**
   * @brief: Method to return the (discretized) OccupancyGrid as a labeled
   * PointCloud.
   * The PointCloud is composed by the centers of each grid cell. Dependend on
   * the contents of grid in a cell, the point representing this cell will be
   * assigned with an associated color and point label.
   * Note: The former contents of output argument 'cloud_representation' will
   * be deleted by this method.
   *
   * @param[out] cloud_representation The OccupancyGrid as a labeled PointCloud.
   **/
  void
  createCloudRepresentation (const LabelCloudPtr grid_cloud);

  static const uint32_t OCCUPIED_LABEL;
  static const uint32_t SPECIAL_FIELD_LABEL;
  static const uint32_t EMPTY_NO_CORR_LABEL;
};

#endif // OCCUPANCY_GRID_H_
