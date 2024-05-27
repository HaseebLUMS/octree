/**
 * Modified from https://github.com/PointCloudLibrary/pcl/blob/master/tools/compute_hausdorff.cpp
*/

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

using PointType = PointXYZ;
using Cloud = PointCloud<PointXYZ>;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s cloud_a.ply cloud_b.ply\n", argv[0]);
}

bool
loadCloud (const std::string &filename, Cloud &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPLYFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const Cloud::ConstPtr &cloud_a, const Cloud::ConstPtr &cloud_b)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  // compare A to B
  pcl::search::KdTree<PointType> tree_b;
  tree_b.setInputCloud (cloud_b);
  float max_dist_a = -std::numeric_limits<float>::max ();
  for (const auto &point : (*cloud_a))
  {
    pcl::Indices indices (1);
    std::vector<float> sqr_distances (1);

    tree_b.nearestKSearch (point, 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_a)
      max_dist_a = sqr_distances[0];
  }

  // compare B to A
  pcl::search::KdTree<PointType> tree_a;
  tree_a.setInputCloud (cloud_a);
  float max_dist_b = -std::numeric_limits<float>::max ();
  for (const auto &point : (*cloud_b))
  {
    pcl::Indices indices (1);
    std::vector<float> sqr_distances (1);

    tree_a.nearestKSearch (point, 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_b)
      max_dist_b = sqr_distances[0];
  }

  max_dist_a = std::sqrt (max_dist_a);
  max_dist_b = std::sqrt (max_dist_b);

  float dist = std::max (max_dist_a, max_dist_b);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
  print_info ("A->B: "); print_value ("%f", max_dist_a);
  print_info (", B->A: "); print_value ("%f", max_dist_b);
  print_info (", Hausdorff Distance: "); print_value ("%f", dist);
  print_info (" ]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute Hausdorff distance between point clouds. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need two PLY files to compute Hausdorff distance.\n");
    return (-1);
  }

  // Load the first file
  Cloud::Ptr cloud_a (new Cloud);
  if (!loadCloud (argv[p_file_indices[0]], *cloud_a))
    return (-1);

  // Load the second file
  Cloud::Ptr cloud_b (new Cloud);
  if (!loadCloud (argv[p_file_indices[1]], *cloud_b))
    return (-1);

  // Compute the Hausdorff distance
  compute (cloud_a, cloud_b);
}
