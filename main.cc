#include "encode.hpp"
#include "decode.hpp"

int main() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;
    Reader.read("./assets/ricardo.ply", *cloud);
    std::vector<double> voxel_sizes = {1};
    for (auto vox : voxel_sizes) {
        OctreeType octree(vox);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        // Compression
        auto [non_negotiable_comp_part, negotiable_comp_part, points_order] = compressOctree(octree);
        auto compressed_colors = compressColors(cloud, points_order);

        // Decompression
        auto [decompressed_centers, side_length] = decompressNonNegotiableBytes(non_negotiable_comp_part);
        auto points = decompressNegotiableBytes(negotiable_comp_part, decompressed_centers, side_length);
        auto decompressed_colors = decompressColors(compressed_colors);

        writeToFile(0, points, decompressed_colors);
        showStats(non_negotiable_comp_part, negotiable_comp_part, compressed_colors);
    }
}