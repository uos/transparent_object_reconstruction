#include <transparent_object_reconstruction/hole_region.h>


HoleRegion::HoleRegion ()
{
  adjoining_grid_border = false;
  holes.clear ();
  nr_border_cells = 0;
  nr_convex_hull_border_cells = 0;
}

HoleRegion::HoleRegion (const HoleRegion& other)
{
  // just copy all members
  this->adjoining_grid_border = other.adjoining_grid_border;
  this->holes = other.holes;  // note: this creates a deep copy
  this->nr_border_cells = other.nr_border_cells;
  this->nr_convex_hull_border_cells = other.nr_convex_hull_border_cells;
}

HoleRegion::~HoleRegion ()
{
  // nothing to do so far
}

void
HoleRegion::push_back (Eigen::Vector2i hole_location)
{
  holes.push_back (hole_location);
}

size_t
HoleRegion::size (void)
{
  return holes.size ();
}

bool
HoleRegion::isAdjacentToBorder (void)
{
  return adjoining_grid_border;
}

void
HoleRegion::setAdjacentToBorder (void)
{
  adjoining_grid_border = true;
}

std::list<Eigen::Vector2i>::const_iterator
HoleRegion::begin (void)
{
  return holes.begin ();
}

std::list<Eigen::Vector2i>::const_iterator
HoleRegion::end (void)
{
  return holes.end ();
}

bool
HoleRegion::operator< (const HoleRegion &other) const
{
  return this->holes.size () < other.holes.size ();
}

void
HoleRegion::incrementBorderCells (void)
{
  this->nr_border_cells++;
}

void
HoleRegion::incrementConvexBorderCells (void)
{
  this->nr_convex_hull_border_cells++;
}

unsigned int
HoleRegion::getNrBorderCells (void)
{
  return this->nr_border_cells;
}

unsigned int
HoleRegion::getNrConvexBorderCells (void)
{
  return this->nr_convex_hull_border_cells;
}
