#include "common.h"
#include "encode.h"
#include "decode.h"

int main() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;
    Reader.read("./../assets/ricardo.ply", *cloud);
    std::vector<double> voxel_sizes = {1};

    for (auto vox : voxel_sizes) {
        OctreeType octree(vox);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();

        // Compression
        auto t1 = std::chrono::high_resolution_clock::now();
        WebpEncDec* color_enc = new WebpEncDec();
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
        auto [decompressed_centers, side_length] = decompressNonNegotiableBytes(non_negotiable_comp_part);
        auto points = decompressNegotiableBytes(negotiable_comp_part, decompressed_centers, side_length);
        auto decompressed_colors = decompressColors(compressed_colors, color_dec);
        auto t4 = std::chrono::high_resolution_clock::now();

        writeToFile("./../output/ricardo.ply", points, decompressed_colors);
        // showStats(non_negotiable_comp_part, negotiable_comp_part, compressed_colors, {t1, t2, t3, t4});
        test();
    }
}