/*
 * Can be used to compute the intersections (in terms of voxels)
 * of a point cloud fused from different frames. The points belonging
 * to each individual point cloud should have a different label (so
 * PointType should be pcl::PointXYZRGBL). It is assumed, that the labels
 * start with label '0' (for the first cloud) and are increased by '1' for
 * each additional pointcloud (TODO: automatically determine the labels)
 *
 * Usuable input can be created via 'fuse_ray_clouds' for example.
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/console/parse.h>

#include <vector>
#include <set>
#include <iostream>

typedef pcl::PointXYZRGBL PointC;
typedef pcl::PointCloud<PointC> CloudC;
typedef CloudC::Ptr CloudCPtr;

const double RESOLUTION = 0.005f;  // 0.5cm
unsigned int MIN_VOXEL_POINTS = 10; // minimal nr of points to look at a voxel
size_t LABEL_ADD_TERM = 15;
static bool write_binary;


void
usage (char** argv)
{
  std::cout << "usage:\n" << argv[0]
    << " -f fused_cloud.pcd"
    << " [-o output_file.pcd]"
    << " [-subset k]"
    << " [--ascii]"
    << " [--all_points]"
    << std::endl;
}

void
buildsubsets_recursive (const std::vector<uint32_t> &indexed_set,
    std::vector<uint32_t> &subset,
    std::vector<std::vector<uint32_t> > &all_subsets,
    size_t subset_write_index = 0, size_t set_read_index = 0)
{
  // recursion base: is the current subset filled completely?
  if (subset_write_index == subset.size ())
  {
    all_subsets.push_back (subset); // add current subset to all subsets
  }
  else
  {
    for (size_t i = set_read_index; i < indexed_set.size (); ++i)
    {
      subset[subset_write_index] = indexed_set[i];
      buildsubsets_recursive (indexed_set, subset, all_subsets,
          subset_write_index + 1, i + 1);
    }
  }
}

/*
 * Creates all subsets of size 'subset_size' from an original set
 * passed via 'labels' and returns them via 'all_subsets'
 */
void
create_all_subsets (const std::set<uint32_t> &labels, size_t subset_size,
    std::vector<std::vector<uint32_t> > &all_subsets,
    std::vector<std::set<uint32_t> > &all_subsets_as_sets)
{
  // remove any lingering contents fomr the output vector
  all_subsets.clear ();
  all_subsets_as_sets.clear ();
  // set up the space for a single subset
  std::vector<uint32_t> subset;
  subset.resize (subset_size);

  // copy all 
  
  // determine the number of labels in cloud
  std::map<uint32_t, size_t> label_stats;
  std::map<uint32_t, size_t>::itertor label_stats_it;
  size_t labeled_points = 0;
  labels from the set into an indexed container
  std::vector<uint32_t> indexed_labels;
  indexed_labels.reserve (labels.size ());
  std::set<uint32_t>::const_iterator label_it = labels.begin ();
  while (label_it != labels.end ())
  {
    indexed_labels.push_back (*label_it++);
  }

  // recursively build all possible subsets of the desired subset_size
  buildsubsets_recursive (indexed_labels, subset, all_subsets);

  // convert all created subsets into 'std::set<>' instances
  all_subsets_as_sets.reserve (all_subsets.size ());
  std::vector<std::vector<uint32_t> >::const_iterator subset_it;
  subset_it = all_subsets.begin ();
  while (subset_it != all_subsets.end ())
  {
    // create the next 'std::set' for the subset
    std::set<uint32_t> subset_as_set;
    subset_as_set.insert (subset_it->begin (), subset_it->end ());
    all_subsets_as_sets.push_back (subset_as_set);
    subset_it++;
  }

  std::cout << "created all desired subsets of size "
    << subset_size << std::endl;
}


bool
isLeafIntersection (const CloudC &leaf_cloud,
    const std::vector<uint32_t> &labels)
{
  // create map for label detection
  std::map<uint32_t, size_t> leaf_labels;
  std::map<uint32_t, size_t>::iterator detected_label;
  std::vector<uint32_t>::const_iterator label_it = labels.begin ();
  while (label_it != labels.end ())
  {
    leaf_labels.insert (std::pair<uint32_t, size_t> (*label_it, 0));
    label_it++;
  }
  std::vector<PointC, Eigen::aligned_allocator<PointC> >::const_iterator p_it;

  // fill the map
  p_it = leaf_cloud.points.begin ();
  while (p_it != leaf_cloud.points.end ())
  {
    detected_label = leaf_labels.find (p_it->label);
    if (detected_label != leaf_labels.end ())
    {
      detected_label->second++;   // increase counter for detected label
    }
    p_it++;
  }

  size_t nr_leaf_points = leaf_cloud.points.size ();
  // TODO: check if the formular makes sense or if 1 point of every label should be sufficient
  size_t min_points_per_label =
    floor (static_cast<double> (nr_leaf_points) /
        static_cast<double> (labels.size () + LABEL_ADD_TERM));
  min_points_per_label = max (static_cast<size_t> (1), min_points_per_label);

  bool is_label_missing = false;
  detected_label = leaf_labels.begin ();
  while (detected_label != leaf_labels.end ())
  {
    if (detected_label->second < min_points_per_label)
    {
      is_label_missing = true;
      return false;
    }
    detected_label++;
  }
  if (!is_label_missing)
    return true;

  return false;
}

bool
isInSubset (const std::set<uint32_t> &subset, const PointC &point)
{
  if (subset.find (point.label) == subset.end ())
  {
    return false;
  }
  return true;
}


int
main (int argc, char** argv)
{
  size_t subset_size;
  int k;
  bool filter_labels = true;
  if (!(pcl::console::find_argument (argc, argv, "-f") > 0))
  {
    usage (argv);
    return EXIT_FAILURE;
  }
  std::string file_name;
  std::string out_file;
  pcl::console::parse_argument (argc, argv, "-f", file_name);
  pcl::PCDReader reader;
  size_t pos;

  if (pcl::console::find_argument (argc, argv, "--ascii") > 0)
  {
    write_binary = false;
  }
  else
  {
    write_binary = true;
  }

  if (pcl::console::find_argument (argc, argv, "-o") > 0)
  {
    pcl::console::parse_argument (argc, argv, "-o", out_file);
    pos = out_file.find (".pcd");
    if (pos != std::string::npos)
    {
      out_file = out_file.substr (0, pos);
    }
  }
  else
  {
    pos = file_name.find (".pcd");
    std::stringstream ss;
    ss << file_name.substr (0, pos) << "_intersection";
    out_file = ss.str ();
  }

  if (pcl::console::find_argument (argc, argv, "--all_points") > 0)
  {
    filter_labels = false;
  }

  
  CloudCPtr cloud (new CloudC);

  int reader_result = reader.read<PointC> (file_name, *cloud);
  std::cout << "reader_result: " << reader_result << std::endl;

  std::set<uint32_t> labels;
  std::vector<PointC, Eigen::aligned_allocator<PointC> >::const_iterator p_it;
  p_it = cloud->points.begin ();
  while (p_it != cloud->points.end ())
  {
    labels.insert (p_it->label);
    p_it++;
  }

  if (labels.size () == 1)
  {
    std::cerr << "Cloud is either unlabeled or contains only points with"
      << " 1 label. Aborting." << std::endl;
    return EXIT_FAILURE;
  }

  CloudCPtr intersec_cloud (new CloudC);

  std::cout << "Nr of different labels: " << labels.size () << std::endl;

  // check (and read if necessary) the desired subset size
  if (pcl::console::find_argument (argc, argv, "-subset") > 0)
  {
    pcl::console::parse_argument (argc, argv, "-subset", k);
  }
  else
  {
    k = labels.size ();
  }

  // check if desired subset size is in legal boundaries
  if (k < 2)
  {
    std::cerr << "Specified subset size (" << k
      << ") is too small. Computing intersection of all labels."
      << std::endl;
    subset_size = labels.size ();
  }
  else if (k > labels.size ())
  {
    std::cerr << "Specified subset size (" << k
      << ") is larger than available number of labels (" << labels.size ()
      << "). Computing intersection of all labels." << std::endl;
    subset_size = labels.size ();
  }
  else
  {
    subset_size = static_cast<size_t> (k);
  }


  std::cout << "subset_size: " << subset_size << std::endl;


  std::set<uint32_t>::const_iterator label_it = labels.begin ();
  
  // just for fun try a small statistic
  std::map<uint32_t, size_t> label_stats;
  std::map<uint32_t, size_t>::iterator label_stats_it;
  size_t labeled_points = 0;
  label_it = labels.begin ();
  std::vector<size_t> filled_count;
  std::vector<size_t> leafs_count;
  while (label_it != labels.end ())
  {
    label_stats.insert (std::pair<uint32_t, size_t> (*label_it, 0));
    label_it++;
  }

  p_it = cloud->points.begin ();

  while (p_it != cloud->points.end ())
  {
    label_stats_it = label_stats.find (p_it->label);
    if (label_stats_it != label_stats.end ())
    {
      label_stats_it->second++;
    }
    p_it++;
  }
   
  label_stats_it = label_stats.begin ();
  while (label_stats_it != label_stats.end ())
  {
    std::cout << "label: " << label_stats_it->first << "\tcount: "
      << label_stats_it->second << std::endl;
    labeled_points += label_stats_it->second;
    label_stats_it++;
  }
  std::cout << "number points:         " << cloud->points.size () << std::endl;
  std::cout << "number labeled points: " << labeled_points
    << std::endl;
  std::cout << "difference:            "
    << cloud->points.size () - labeled_points << std::endl;

  label_it = labels.begin ();

  pcl::PCDWriter writer;
  std::stringstream ss;

  // create an octree for the pointcloud
  pcl::octree::OctreePointCloud<PointC> octree (RESOLUTION);
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  // iterate over all leaf nodes and determine which leaves are filled with
  // points from all labels
  pcl::octree::OctreePointCloud<PointC>::LeafNodeIterator leaf_it;
  leaf_it = octree.leaf_begin ();
  pcl::PointIndices::Ptr leaf_point_indices (new pcl::PointIndices);
  pcl::PointCloud<PointC>::Ptr leaf_cloud (new pcl::PointCloud<PointC>);
  pcl::ExtractIndices<PointC> extract;
  CloudC::Ptr labeled_voxel (new CloudC);
  size_t leaf_counter = 0;
  size_t filled_leaf_counter = 0;
  size_t leaf_points;

  std::vector<PointC, Eigen::aligned_allocator<PointC> > bla;
  int nr_leafs = octree.getOccupiedVoxelCenters (bla);

  std::cout << "nr_leafs: " << nr_leafs << std::endl;

  std::list<CloudC> filled_leaves;
  // set the input cloud for the point extraction object
  extract.setInputCloud (cloud);

  // go over all leaves to fill stats and store all filled leaves once
  while (*++leaf_it)
  {
    // remove old indices
    leaf_point_indices->indices.clear ();
    // increase counter for leaves (just for stats)
    leaf_counter++;

    // retrieve the container for the current leaf
    pcl::octree::OctreeContainerPointIndices &container = leaf_it.getLeafContainer ();
    container.getPointIndices (leaf_point_indices->indices);

    // store the number of points inside the leaf (just for stats)
    leafs_count.push_back (leaf_point_indices->indices.size ());

    // check if there are enough points inside the leaf to consider it filled
    if (leaf_point_indices->indices.size () > MIN_VOXEL_POINTS)
    {
      // store number of points (just for stats)
      filled_count.push_back (leaf_point_indices->indices.size ());
      // increase number of filled leaves (just for stats)
      filled_leaf_counter++;
      // retrieve the points from the point indices
      extract.setIndices (leaf_point_indices);
      extract.filter (*leaf_cloud);
      // store the filled leaf in the list of all filled leaves
      filled_leaves.push_back (*leaf_cloud);
    }
  }

  // print the stats
  std::cout << "leaf_counter: " << leaf_counter << std::endl;
  std::cout << "filled_leaf_counter: " << filled_leaf_counter << std::endl;

  size_t filled_points = 0;
  size_t l_points = 0;
  std::vector<size_t>::const_iterator it = filled_count.begin ();
  while (it != filled_count.end ())
  {
    filled_points += *it;
    it++;
  }
  it = leafs_count.begin ();
  while (it != leafs_count.end ())
  {
    l_points += *it;
    it++;
  }

  double l_avg_p = static_cast<double> (l_points) /
    static_cast<double> (leafs_count.size ());
  double f_avg_p = static_cast<double> (filled_points) /
    static_cast<double> (filled_count.size ());

  std::cout << "l_avg_p: " << l_avg_p << std::endl;
  std::cout << "f_avg_p: " << f_avg_p << std::endl;

  // compute the desired subsets for intersection computation
  std::vector<std::vector<uint32_t> > all_subsets;
  std::vector<std::set<uint32_t> > all_subsets_as_sets;
  create_all_subsets (labels, subset_size, all_subsets, all_subsets_as_sets);

  std::list<CloudC>::const_iterator filled_leaf_it;

  // loop over all subsets
  for (size_t i = 0; i < all_subsets.size (); ++i)
  {
    // set the current subset
    std::vector<uint32_t> &current_subset = all_subsets[i];
    std::set<uint32_t> &curr_subset_as_set = all_subsets_as_sets[i];
    // clear the intersection cloud and reserve enough memory
    intersec_cloud->points.clear ();
    intersec_cloud->points.reserve (cloud->points.size ());

    // set the leaf iterator to the first filled leaf
    filled_leaf_it = filled_leaves.begin ();

    //loop over all leaves and check if they belong to the intersection
    while (filled_leaf_it != filled_leaves.end ())
    {
      // check if the current leaf is part of the intersection
      if (isLeafIntersection (*filled_leaf_it, current_subset))
      {
        p_it = filled_leaf_it->points.begin ();
        while (p_it != filled_leaf_it->points.end ())
        {
          // if desired only add the points of the current subset
          if (filter_labels)
          {
            // check it the leaf point belongs to the current subset
            if (isInSubset (curr_subset_as_set, *p_it))
            {
              intersec_cloud->points.push_back (*p_it);
            }
          }
          // otherwise just add all points
          else
          {
            intersec_cloud->points.push_back (*p_it);
          }
          p_it++;
        }
      }
      filled_leaf_it++;
    }

    // generate filename (with used labels)
    ss.str ("");
    ss << out_file;
    for (size_t i = 0; i < current_subset.size (); ++i)
    {
      ss << "_" << current_subset[i];
    }
    ss << ".pcd";

    // store the intersection if it contains any points
    if (intersec_cloud->points.size () > 0)
    {
      intersec_cloud->width = intersec_cloud->points.size ();
      intersec_cloud->height = 1;
      writer.write<PointC> (ss.str (), *intersec_cloud, write_binary);
      std::cout << "Intersection '" << ss.str () << "' contains "
        << intersec_cloud->points.size () << " points." << std::endl;
    }
    else
    {
      std::cout << "Intersection '" << ss.str ()
        << "' doesn't contain any points." << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
