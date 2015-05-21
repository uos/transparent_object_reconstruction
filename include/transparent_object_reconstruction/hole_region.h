#ifndef HOLE_REGION_H_
#define HOLE_REGION_H_

#include <Eigen/Core>

#include <iostream>
#include <list>

/**
 * Just a small class used to manage the properties of a hole detected
 * in an OccupancyGrid (although that is not a dependency). Contains
 * information about the grid cells that compose the hole, if it touches
 * the borders of the OccupancyGrid and the number of cells that comprise
 * the border of the hole and its convex hull respectively.
 **/
class HoleRegion
{
  // denoting if this hole touches the border of the occupancy grid
  bool adjoining_grid_border;
  // list containing the coords of the 2D grid cells that make up the hole
  std::list<Eigen::Vector2i> holes;
  // number grid cells the belong to the border of the OccupancyGrid
  unsigned int nr_border_cells;
  // number of cells in convex hull of the (partially) enclosing planar surface
  unsigned int nr_convex_hull_border_cells;

  public:

  /**
   * Default constructor for an empty HoleRegion. Initially each HoleRegion
   * is empty (i.e., no grid cells belong to this hole, the number of
   * border cells and the number of convex border cells is 0 and the
   * HoleRegion does not adjoin the grid border.
   **/
  HoleRegion ();

  /**
   * Copy constructor. Performs a deep copy of the given HoleRegion.
   *
   * @param[in] other The HoleRegion to be copied
   **/
  HoleRegion (const HoleRegion& other);

  /**
   * Destructor (which doesn't really do anything since there is no
   * dynamic memory allocation).
   **/
  ~HoleRegion ();

  /**
   * Method to add a new grid cell to the current HoleRegion. The grid
   * cell is specified via its integer coordinates.
   *
   * @param[in] hole_location The coordinates of the grid cell that is
   *    added to the current HoleRegion.
   **/
  void
  push_back (Eigen::Vector2i hole_location);

  /**
   * Function to return the size of the current HoleRegion. Size in this
   * case refers to the number of grid cells that contribute to the
   * current HoleRegion (since each grid cell corresponds to an area this
   * also can be used to determine the size in terms of area that the
   * current HoleRegion occupies).
   *
   * @return The number of grid cells that contribute to the current
   *   HoleRegion.
   **/
  size_t
  size (void);

  /**
   * Getter function to indicate if the current HoleRegion is in contact
   * with the border of the occupancy grid.
   *
   * @return true, if the HoleRegion contacts the border of the occupancy
   *   grid; false otherwise
   **/
  bool
  isAdjacentToBorder (void);

  /**
   * Setter method to indicate that the HoleRegion is in contact with the
   * border of the occupancy grid. Note that once this method is called it
   * can't be revoked again, i.e., if a HoleRegion is once set adjacent
   * to the border of the occupancy grid it will stay that way.
   **/
  void
  setAdjacentToBorder (void);

  /**
   * Setter function to increase the number of border cells.
   **/
  void
  incrementBorderCells (void);
  /**
   * Setter function to increase the number of convex border cells.
   **/
  void
  incrementConvexBorderCells (void);

  /**
   * Function to retrieve the number of cells that form the border of the
   * current HoleRegion.
   *
   * @return The number of grid cells forming the border of the current
   *   HoleRegion.
   **/
  unsigned int
  getNrBorderCells (void);
  /**
   * Function to retrieve the number of cells that form the convex border
   * of the current HoleRegion.
   *
   * @return The number of grid cells forming the convex border of the
   *    current HoleRegion.
   **/
  unsigned int
  getNrConvexBorderCells (void);

  /**
   * Constant iterator pointing to the beginning of the list holding all
   * grid cell coordinates that belong to the current HoleRegion
   **/
  std::list<Eigen::Vector2i>::const_iterator
  begin (void);
  /**
   * Constant iterator pointing to the end of the list holding all
   * grid cell coordinates that belong to the current HoleRegion
   **/
  std::list<Eigen::Vector2i>::const_iterator
  end (void);

  /**
   * Comparator function to induce ordering on HoleRegions. HoleRegions
   * are compared according to their size, i.e., the corrent HoleRegion
   * is smaller if it is composed of less grid cells than the HoleRegion
   * given via input argument 'other'.
   *
   * @param[in] other The HoleRegion to compare the current HoleRegion with
   * @return true if the current HoleRegion is smaller than 'other'; false
   *    otherwise
   **/
  bool
  operator< (const HoleRegion &other) const;

};

#endif // HOLE_REGION_H_
