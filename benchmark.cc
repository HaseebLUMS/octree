#include <iostream>
#include <queue>
#include <vector>

#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZ PointType;  // Change the point type if necessary
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::octree::OctreePointCloud<PointType> OctreeType;

void performBFS(OctreeType& octree, double voxel_size)
{
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator it = octree.depth_begin();
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator it_end = octree.depth_end();

    std::unordered_map<int, long long int> node_counts_per_level;    
    
    while (it != it_end) {
        node_counts_per_level[it.getCurrentOctreeDepth()]++;
        it++;
    }

    long long int total_nodes = 0;
    long long int nodes_at_levels_with_leaves_cardinality = 0;
    long long int nodes_at_last_level = node_counts_per_level[octree.getTreeDepth()];

    for (auto [k, v]: node_counts_per_level) {
        if (v == octree.getLeafCount()) {
            nodes_at_levels_with_leaves_cardinality += v;
        }
        total_nodes += v;
    }

    std::cout << "[voxel size: "<< voxel_size << "]Total Nodes: " << total_nodes << std::endl;
    std::cout << "[voxel size: "<< voxel_size << "]Nodes at last(" << octree.getTreeDepth() << "th) Level : " << nodes_at_last_level << std::endl;
    std::cout << "[voxel size: "<< voxel_size << "]Nodes at PointsCount Levels: " << nodes_at_levels_with_leaves_cardinality << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
}

int main() {
    // std::vector<std::string> files = {"prettygirl.ply", "ricardo.ply", "soldier.ply"};
    std::vector<std::string> files = {"thaidancer.ply"};

    for (auto dataset_name : files) {
        std::string dataset = "/Users/haseeb/Documents/GitHub/octree/assets/" + dataset_name;
        std::cout << "Using Dataset: " << dataset << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PLYReader Reader;
        Reader.read(dataset, *cloud);
        std::vector<double> voxel_sizes = {2, 1.5, 1, 0.5, 0.1, 0.01};
        for (auto ele : voxel_sizes) {
            OctreeType octree(ele);
            octree.setInputCloud(cloud);
            octree.defineBoundingBox();
            octree.addPointsFromInputCloud();
            performBFS(octree, ele);
        }
    }
}