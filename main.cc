#include "encode.hpp"
#include "decode.hpp"

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read("./assets/ricardo.ply", *cloud);
    std::vector<double> voxel_sizes = {0.44};
    for (auto ele : voxel_sizes) {
        OctreeType octree(ele);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        auto compressed_octree = compressOctree(octree);
        auto decompressed_octree = decompressOctree(compressed_octree);
        write_to_file("./output/output.ply", decompressed_octree);
    }
}