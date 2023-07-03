#include "encode.hpp"

int main() {
    std::vector<std::string> files = {"ricardo.ply"};

    for (auto dataset_name : files) {
        std::string dataset = "/Users/haseeb/Documents/GitHub/octree/assets/" + dataset_name;
        std::cout << "Using Dataset: " << dataset << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PLYReader Reader;
        Reader.read(dataset, *cloud);
        std::vector<double> voxel_sizes = {0.44};
        for (auto ele : voxel_sizes) {
            OctreeType octree(ele);
            octree.setInputCloud(cloud);
            octree.defineBoundingBox();
            octree.addPointsFromInputCloud();
            compressOctree(octree);
        }
    }
}