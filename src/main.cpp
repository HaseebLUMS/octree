#include <filesystem>
#include <sstream> 

#include "common.h"
#include "encode.h"
#include "decode.h"

const char* dataset = "ricardo.ply";

int main() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;

    Reader.read(std::string("./../assets/") + dataset, *cloud);
    std::vector<double> voxel_sizes = {1};

    for (auto vox : voxel_sizes) {
        OctreeType octree(vox);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        // Compression
        auto t1 = std::chrono::high_resolution_clock::now();
        WebpEncDec* color_enc = new WebpEncDec();
        // JpegEncDec* color_enc = new JpegEncDec();
        auto [non_negotiable_comp_part, negotiable_comp_part, points_order] = compressOctree(octree);
        auto compressed_colors = compressColors(cloud, points_order, color_enc);
        auto t2 = std::chrono::high_resolution_clock::now();

        /**
         * Following items need to be sent to the receiver so that a volumetric frame may be constructed:
         * non_negotiable_comp_part, negotiable_comp_part, compressed_colors
        */

        // Decompression
        auto t3 = std::chrono::high_resolution_clock::now();
        WebpEncDec* color_dec = new WebpEncDec();
        // JpegEncDec* color_dec = new JpegEncDec();
        auto [decompressed_centers, side_length] = decompressNonNegotiableBytes(non_negotiable_comp_part);
        auto points = decompressNegotiableBytes(negotiable_comp_part, decompressed_centers, side_length);
        auto decompressed_colors = decompressColors(compressed_colors, color_dec);
        auto t4 = std::chrono::high_resolution_clock::now();

        // std::vector<double> drops = {0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 0.6, 0.7};
        std::vector<double> drops = {0.0};

        std::string path = std::string("./../output/") + dataset + std::string("/");
        if (!std::filesystem::exists(path)) {
            if (std::filesystem::create_directory(path)) {
                std::cout << "Directory created successfully: " << path << std::endl;
            } else {
                std::cerr << "Failed to create directory: " << path << std::endl;
            }
        }

        for (auto d : drops) {
            std::ostringstream s;
            s << path << d;
            std::string str = s.str(); 

            writeToFile(str + std::string(".ply"), points, decompressed_colors, d, false);
            // writeToFile(str + std::string("_block.ply"), points, decompressed_colors, d, true);
        }

        showStats(non_negotiable_comp_part, negotiable_comp_part, compressed_colors, {t1, t2, t3, t4});
        test();
    }
}