#include "encode.hpp"
#include "decode.hpp"

int main() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;
    Reader.read("./assets/ricardosmall.ply", *cloud);
    std::vector<double> voxel_sizes = {1};
    for (auto vox : voxel_sizes) {
        OctreeType octree(vox);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        std::vector<double> lost_probabilities = {0, 0.01, 0.1, 0.5, 1, 10, 50, 100};
        for (auto lp : lost_probabilities) {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto [compressed_octree, points_order] = compressOctree(octree, lp);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << "Geometry Compression time: " << duration << " milliseconds" << std::endl;

            start_time = std::chrono::high_resolution_clock::now();
            auto compressed_colors = compressColors(cloud, points_order);
            end_time = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << "Color Compression time: " << duration << " milliseconds" << std::endl;

            double total_bytes = compressed_octree.negotiable_bytes.size()+compressed_octree.non_negotiable_bytes.size();
            std::cout << "Non Negotiable Bytes: " << compressed_octree.non_negotiable_bytes.size() << std::endl;
            std::cout << "Negotiable Bytes: " << compressed_octree.negotiable_bytes.size() << std::endl;
            std::cout << "Total Bytes: " << total_bytes << std::endl;
            std::cout << "Negotiable share: " << (100.0*compressed_octree.negotiable_bytes.size())/total_bytes << std::endl;

            // start_time = std::chrono::high_resolution_clock::now();
            // auto decompressed_octree = decompressOctree(compressed_octree);
            // end_time = std::chrono::high_resolution_clock::now();
            // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // std::cout << "Geometry Decompression time: " << duration << " milliseconds" << std::endl;

            // start_time = std::chrono::high_resolution_clock::now();
            // auto decompressed_colors = decompressColors(compressed_colors);
            // end_time = std::chrono::high_resolution_clock::now();
            // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // std::cout << "Color Decompression time: " << duration << " milliseconds" << std::endl;

            // writeToFile(lp, decompressed_octree, decompressed_colors);
            break;
        }
    }
}