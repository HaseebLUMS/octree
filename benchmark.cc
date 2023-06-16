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

void performBFS(OctreeType& octree)
{
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator it = octree.depth_begin();
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator it_end = octree.depth_end();

    std::unordered_map<int, long long int> node_counts_per_level;    
    
    while (it != it_end) {
        node_counts_per_level[it.getCurrentOctreeDepth()]++;
        it++;
    }

    long long int sum_of_nodes = 0;
    long long int sum_of_leaves = 0;

    long long int nodes_at_last_level = node_counts_per_level[octree.getTreeDepth()];

    // std::cout << "node_counts_per_level:" << std::endl;
    for (auto [k, v]: node_counts_per_level) {
        // std::cout << "k: " << k << " v: " << v << std::endl;
        sum_of_nodes += v;
    }

    std::cout << "Total Nodes: " << sum_of_nodes << std::endl;
    std::cout << "Nodes at " << octree.getTreeDepth() << "th Level : " << nodes_at_last_level << std::endl;
    float share_of_last_level = 100 * ((float)nodes_at_last_level / sum_of_nodes);
    std::cout << std::endl;
    std::cout << std::endl;
    // std::cout << "Last Level's share: " << share_of_last_level << "%" << std::endl;
    // std::cout << "Leaf Nodes' share: " << 100* ((float)octree.getLeafCount() / sum_of_nodes) << "%" << std::endl;
    // std::cout << std::endl;
    // std::cout << std::endl;
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    // Reader.read("/Users/haseeb/Documents/GitHub/octree/assets/soldier.ply", *cloud);
    Reader.read("/Users/haseeb/Documents/GitHub/octree/assets/soldier.ply", *cloud);

    std::vector<double> voxel_sizes = {2, 1.5, 1, 0.5, 0.1, 0.01};
    for (auto ele : voxel_sizes) {
        OctreeType octree(ele);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        int nodeCount = octree.getLeafCount();
        // std::cout << "Total leaves: " << nodeCount << std::endl;
        // std::cout << "TotalLevels: " << octree.getTreeDepth() << std::endl;
        performBFS(octree);
    }
    
}