#include <transparent_object_reconstruction/occupancy_grid.h>


size_t OccupancyGrid::min_hole_region_size = 10;
float OccupancyGrid::on_plane_threshold = 0.02f;
const uint32_t OccupancyGrid::EMPTY_NO_CORR_LABEL = std::numeric_limits<uint32_t>::max ();
const uint32_t OccupancyGrid::SPECIAL_FIELD_LABEL = std::numeric_limits<uint32_t>::max () - 1;
const uint32_t OccupancyGrid::OCCUPIED_LABEL = std::numeric_limits<uint32_t>::max () - 2;


OccupancyGrid::OccupancyGrid (pcl::VoxelGrid<PointType> &voxel_grid, CloudPtr cloud,
    std::vector<Vector4fVec> &transformed_plane_equations,
    size_t index)
{
  this->voxel_grid = voxel_grid;
  this->cloud = cloud;
  this->transformed_plane_equations = transformed_plane_equations;
  this->index = index;

  min_box = this->voxel_grid.getMinBoxCoordinates ();
  max_box = this->voxel_grid.getMaxBoxCoordinates ();
  x_dim = max_box[0] - min_box[0] + 1;
  y_dim = max_box[1] - min_box[1] + 1;
  occupancy_grid = new grid_values*[x_dim];
  for (size_t i = 0; i < x_dim; ++i)
  {
    occupancy_grid[i] = new grid_values[y_dim];
  }

  pcl::getMinMax3D (*(this->cloud), min_point, max_point);
  leaf_size = this->voxel_grid.getLeafSize ();

  // allocate and initialize auxiliary grid
  visited = new bool*[x_dim];
  for (size_t i = 0; i < x_dim; ++i)
  {
    visited[i] = new bool[y_dim];
    for (size_t j = 0; j < y_dim; ++j)
    {
      visited[i][j] = false;
    }
  }
}

OccupancyGrid::~OccupancyGrid (void)
{
  for (size_t i = 0; i < x_dim; ++i)
  {
    delete[] occupancy_grid[i];
    delete[] visited[i];
  }
  delete[] occupancy_grid;
  delete[] visited;
}

void
OccupancyGrid::fillGrid (void)
{
  int grid_cell_index;
  grid_values grid_value_result;
  // iterator vector over the voxel grid
  Eigen::Vector3i it_vec (min_box);
  // indices for the occupancy grids
  size_t occ_x, occ_y;
  occ_x = occ_y = 0;

  // initialize the 3D (4D) point to test for the occupancy of the grid
  test_point = Eigen::Vector4f (min_point[0] + 0.5f * leaf_size[0],
      min_point[1] + 0.5f * leaf_size[1], min_point[2], 1.0f);

  // determine which cells are empty / occupied / behind other planes
  for (int grid_x = min_box[0]; grid_x < max_box[0]; ++grid_x)
  {
    // set the x coordinate of the iterator vector in voxel grid coords
    it_vec[0] = grid_x;
    // reset y index for the occupancy grid
    occ_y = 0;
    // reset y coordinate of the test point
    test_point[1] = min_point[1] + 0.5f * leaf_size[1];

    for (int grid_y = min_box[1]; grid_y < max_box[1]; ++ grid_y)
    {
      // ste the y coordinate of the iterator vector
      it_vec[1] = grid_y;

      // retrieve the index of the grid centroid at the iterator vector
      grid_cell_index = voxel_grid.getCentroidIndexAt (it_vec);
      // calc the value for the grid cell
      grid_value_result = calcGridValue (test_point,
          transformed_plane_equations, index, grid_cell_index);

      // store the value at the appropriate place in the occupancy grid
      occupancy_grid[occ_x][occ_y] = grid_value_result;
      // update the y index for the occupancy grid
      occ_y++;
      // update the y coordinate of the test point
      test_point[1] += leaf_size[1];
    }
    // update the x index of the occupancy grid
    occ_x++;
    // update the x coordinate of the test point
    test_point[0] += leaf_size[0];
  }

  // set the borders of the occupancy grid
  for (size_t i = 0; i < x_dim; ++i)
  {
    occupancy_grid[i][0] = grid_border;
    occupancy_grid[i][y_dim - 1] = grid_border;
  }
  for (size_t j = 0; j < y_dim; ++j)
  {
    occupancy_grid[0][j] = grid_border;
    occupancy_grid[x_dim - 1][j] = grid_border;
  }
}

void
OccupancyGrid::addConvHullLines (const CloudPtr hull_lines)
{
  Eigen::Vector3i grid_coords;
  int occ_x, occ_y; // indices for the occupancy grid

  Cloud::VectorType::const_iterator it;

  it = hull_lines->points.begin ();
  while (it != hull_lines->points.end ())
  {
    grid_coords = voxel_grid.getGridCoordinates (it->x, it->y, it->z);
    occ_x = grid_coords[0] - min_box[0];
    occ_y = grid_coords[1] - min_box[1];

    if (occupancy_grid[occ_x][occ_y] == grid_empty)
      occupancy_grid[occ_x][occ_y] = grid_conv_hull_border;
    it++;
  }
}

grid_values**
OccupancyGrid::retrieveOccupancyGrid (size_t &x_dim,
    size_t &y_dim)
{
  x_dim = this->x_dim;
  y_dim = this->y_dim;
  return this->occupancy_grid;
}

PointType
OccupancyGrid::getMinPoint (void)
{
  PointType _min_point;
  _min_point.x = min_point[0];
  _min_point.y = min_point[1];
  _min_point.z = min_point[2];
  return _min_point;
}

PointType
OccupancyGrid::getMaxPoint (void)
{
  PointType _max_point;
  _max_point.x = max_point[0];
  _max_point.y = max_point[1];
  _max_point.z = max_point[2];
  return _max_point;
}

Eigen::Vector3f
OccupancyGrid::getLeafSize (void)
{
  return this->leaf_size;
}

void
OccupancyGrid::recursiveGrowHoleRegion (size_t x_index, size_t y_index,
    HoleRegion* hole_region)
{
  // first recursion base (don't use already visited cells)
  if (visited[x_index][y_index])
    return;
  // mark cell as visited
  visited[x_index][y_index] = true;
  // second recursion base (only add empty cells)
  if (occupancy_grid[x_index][y_index] != grid_empty)
  {
    // mark the HoleRegion if it touches the grid border
    if (occupancy_grid[x_index][y_index] == grid_border)
    {
      hole_region->setAdjacentToBorder ();
    }
    return;
  }
  // add the grid cell to the hole region
  hole_region->push_back (Eigen::Vector2i (x_index, y_index));
  // recursive call for neighboring cells (using von Neumann neighborhood)
  recursiveGrowHoleRegion (x_index - 1, y_index, hole_region);
  recursiveGrowHoleRegion (x_index + 1, y_index, hole_region);
  recursiveGrowHoleRegion (x_index, y_index - 1, hole_region);
  recursiveGrowHoleRegion (x_index, y_index + 1, hole_region);
}

HoleRegion*
OccupancyGrid::createHoleRegion (size_t x_index, size_t y_index)
{
  if (occupancy_grid[x_index][y_index] != grid_empty ||
      visited[x_index][y_index])
  {
    // return null pointer (to indicate empty hole region)
    return 0;
  }
  HoleRegion* hole_region (new HoleRegion);
  recursiveGrowHoleRegion (x_index, y_index, hole_region);
  std::cout << "hole_region->size (): " << hole_region->size () << std::endl;
  return hole_region;
}

void
OccupancyGrid::createHoleRegions (std::vector<HoleRegion> &hole_regions)
{
  // remove any lingering content from hole_regions
  hole_regions.clear ();
  HoleRegion *hole_region;

  for (size_t i = 0; i < x_dim; ++i)
  {
    for (size_t j = 0; j < y_dim; ++j)
    {
      // only investigate further if cell was not already visited
      if (!visited[i][j])
      {
        // check if the cell is empty
        if (occupancy_grid[i][j] == grid_empty)
        {
          hole_region = createHoleRegion (i, j);
          // check if a hole region was created from this cell
          if (hole_region != 0)
          {
            // check if hole_region fulfills all requirements
            if (!hole_region->isAdjacentToBorder () &&
                hole_region->size () >= min_hole_region_size)
            {
              hole_regions.push_back (*hole_region);
            }
          }
        }
        // mark cell as visited
        visited[i][j] = true;
      }
    }
  }
}

grid_values
OccupancyGrid::calcGridValue (const Eigen::Vector4f &test_point,
    const std::vector<Vector4fVec> &transformed_plane_equations,
    size_t index, int grid_index)
{
  float dot_prod;
  bool on_plane = false;
  bool behind_plane = false;
  for (size_t k = 0; k < transformed_plane_equations[index].size (); ++k)
  {
    if (k != index)
    {
      dot_prod = test_point.dot (transformed_plane_equations[index][k]);
      // check if point lies on plane
      if (fabs (dot_prod) < on_plane_threshold)
      {
        on_plane = true;
        break;
      }
      // check if point lies behin plane
      else if (dot_prod < -on_plane_threshold)
      {
        behind_plane = true;
        break;
      }
    }
  }
  if (on_plane)
    return grid_plane_border;
  else if (behind_plane)
    return grid_behind_plane;
  else if (grid_index != -1)
    return grid_occupied;
  return grid_empty;
}

void
OccupancyGrid::setMinHoleRegionSize (size_t min_size)
{
  min_hole_region_size = min_size;
}

void
OccupancyGrid::setOnPlaneThreshold (float threshold)
{
  on_plane_threshold = threshold;
}

size_t
OccupancyGrid::getMinHoleRegionSize (void)
{
  return min_hole_region_size;
}

float
OccupancyGrid::getOnPlaneThreshold (void)
{
  return on_plane_threshold;
}

void
OccupancyGrid::checkHoleRegionBorder (HoleRegion &hole_region)
{
  // allocate temporary field for border check
  bool** tmp_visited = new bool*[x_dim];
  for (size_t i = 0; i < x_dim; ++i)
  {
    tmp_visited[i] = new bool[y_dim];
    for (size_t j = 0; j < y_dim; ++j)
    {
      tmp_visited[i][j] = false;
    }
  }
  size_t index_x, index_y;
  // get one hole as seed from the hole region
  Eigen::Vector2i seed = *(hole_region.begin ());
  index_x = seed[0];
  index_y = seed[1];

  recursiveCheckHoleRegionBorder (index_x, index_y, hole_region,
      tmp_visited);

  // release temporary field again
  for (size_t i = 0; i < x_dim; ++i)
  {
    delete[] tmp_visited[i];
  }
  delete[] tmp_visited;
}

void
OccupancyGrid::recursiveCheckHoleRegionBorder (size_t x_index,
    size_t y_index, HoleRegion &hole_region, bool** checked)
{
  if (checked[x_index][y_index])
    return;
  checked[x_index][y_index] = true;
  if (occupancy_grid[x_index][y_index] != grid_empty)
  {
    hole_region.incrementBorderCells ();
    if (occupancy_grid[x_index][y_index] == grid_conv_hull_border)
    {
      hole_region.incrementConvexBorderCells ();
    }
    return;
  }
  recursiveCheckHoleRegionBorder (x_index - 1, y_index, hole_region, checked);
  recursiveCheckHoleRegionBorder (x_index + 1, y_index, hole_region, checked);
  recursiveCheckHoleRegionBorder (x_index, y_index - 1, hole_region, checked);
  recursiveCheckHoleRegionBorder (x_index, y_index + 1, hole_region, checked);
}

void
OccupancyGrid::createCloudRepresentation (const LabelCloudPtr grid_cloud)
{
  // remove old contents from output variable
  grid_cloud->points.clear ();

  grid_cloud->width = x_dim;
  grid_cloud->height = y_dim;
  grid_cloud->points.reserve (x_dim * y_dim);

  bool found_corr_hole_region;
  unsigned int nr_empty_cells = 0;

  //TODO: make this more efficient...
  std::vector<HoleRegion> all_hole_regions;
  this->createHoleRegions  (all_hole_regions);

  std::list<Eigen::Vector2i>::const_iterator cell_iterator;
  std::list<Eigen::Vector2i>::const_iterator end_iterator;
  Eigen::Vector2i current_cell_coords;
  LabelPoint lp;

  for (size_t y = 0; y < y_dim; ++y)
  {
    for (size_t x = 0; x < x_dim; ++x)
    {
      switch (occupancy_grid[x][y])
      {
        case grid_empty:
          break;
      }
      // empty grid cell
      if (occupancy_grid[x][y] == grid_empty)
      {
        found_corr_hole_region = false;
        nr_empty_cells++;
        current_cell_coords = Eigen::Vector2i(x, y);
        // check if belongs to any hole region
        for (size_t hole_region_index = 0;
            hole_region_index < all_hole_regions.size ();
            ++hole_region_index)
        {
          cell_iterator = all_hole_regions[hole_region_index].begin ();
          end_iterator = all_hole_regions[hole_region_index].end ();
          // go over all cells of the current hole_region
          while (cell_iterator != end_iterator)
          {
            // check if it belongs to the current hole region
            if (current_cell_coords == *cell_iterator++)
            {
              lp.label = static_cast<uint32_t> (hole_region_index);
              lp.r = lp.g = 0;
              lp.b = 255;
              found_corr_hole_region = true;
              break;
            }
          }
          if (found_corr_hole_region)
          {
            break;
          }
        }
        // if it doesn't belong to any hole_region attach special label // TODO: max value?
        if (!found_corr_hole_region)
        {
          lp.label = OccupancyGrid::EMPTY_NO_CORR_LABEL;
          lp.r = lp.g = lp.b = 100;
        }
      }
      else if (occupancy_grid[x][y] == grid_border)
      {
        lp.label = OccupancyGrid::SPECIAL_FIELD_LABEL;
        lp.r = 255;
        lp.g = lp.b = 0;
      }
      else if (occupancy_grid[x][y] == grid_plane_border)
      {
        lp.label = OccupancyGrid::SPECIAL_FIELD_LABEL;
        lp.r = lp.b = 0;
        lp.g = 255;
      }
      else if (occupancy_grid[x][y] == grid_behind_plane)
      {
        lp.label = OccupancyGrid::SPECIAL_FIELD_LABEL;
        lp.r = 0;
        lp.g = lp.b = 255;
      }
      else if (occupancy_grid[x][y] == grid_conv_hull_border)
      {
        lp.label = OccupancyGrid::SPECIAL_FIELD_LABEL;
        lp.r = lp.g = 255;
        lp.b = 0;
      }
      else
      {
        lp.label = OccupancyGrid::OCCUPIED_LABEL;
        lp.r = lp.g = lp.b = 255;
      }
      // TODO: get coordinates
      // add point to point cloud
      grid_cloud->points.push_back (lp);
    }
  }
}
