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

        auto [non_negotiable_comp_part, negotiable_comp_part, points_order] = compressOctree(octree);
        auto compressed_colors = compressColors(cloud, points_order);

        double total_bytes = negotiable_comp_part.negotiable_bytes.size() + non_negotiable_comp_part.non_negotiable_bytes.size();
        std::cout << "Non Negotiable Bytes: " << non_negotiable_comp_part.non_negotiable_bytes.size() << std::endl;
        std::cout << "Negotiable Bytes: " << negotiable_comp_part.negotiable_bytes.size() << std::endl;
        std::cout << "Total Bytes: " << total_bytes << std::endl;
        std::cout << "Negotiable share: " << (100.0*negotiable_comp_part.negotiable_bytes.size())/total_bytes << std::endl;

        auto [decompressed_centers, side_length] = decompressNonNegotiableBytes(non_negotiable_comp_part);
        auto points = decompressNegotiableBytes(negotiable_comp_part, decompressed_centers, side_length);

        auto decompressed_colors = decompressColors(compressed_colors);

        writeToFile(0, points, decompressed_colors);
    }
}