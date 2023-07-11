#include "encode.hpp"
#include "decode.hpp"

int main() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;
    Reader.read("./assets/ricardo.ply", *cloud);
    std::vector<double> voxel_sizes = {0.1};
    for (auto vox : voxel_sizes) {
        OctreeType octree(vox);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        std::vector<double> lost_probabilities = {0, 0.01, 0.1, 0.5, 1, 10, 50, 100};
        for (auto lp : lost_probabilities) {
            auto compressed_octree = compressOctree(octree, lp);
            auto compressed_colors = compressColors(cloud);

            auto decompressed_octree = decompressOctree(compressed_octree);
            auto decompressed_colors = decompressColors(compressed_colors);
            writeToFile(lp, decompressed_octree, decompressed_colors);
            break;
        }
    }
}