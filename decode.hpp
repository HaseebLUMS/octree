#include "common.h"

bool isLastLevel(int curr_node_index, int nodes_in_last_level, int total_nodes) {
    int starting_index_of_last_level = total_nodes - nodes_in_last_level;
    return curr_node_index >= starting_index_of_last_level;
}

std::vector<Eigen::Vector4f> decompressOctree(compressedOctree compressed_octree) {
    auto compressed_bytes = compressed_octree.bytes;
    
    Eigen::Vector4f* centers = (Eigen::Vector4f*) malloc(
        compressed_octree.num_of_leaves*sizeof(Eigen::Vector4f));

    Eigen::Vector4f* next_centers = (Eigen::Vector4f*) malloc(
        compressed_octree.num_of_leaves*sizeof(Eigen::Vector4f));

    centers[0].x() = compressed_octree.root_center[0];
    centers[0].y() = compressed_octree.root_center[1];
    centers[0].z() = compressed_octree.root_center[2];
    centers[0].w() = 1.0;

    int count_bytes = 0;
    int count_centers = 1;
    auto current_side_length = compressed_octree.root_side_length;

    int total_centers = 1;
    while(count_bytes < compressed_bytes.size()) {
        int count_next_centers = 0;

        for(int i = 0; i < count_centers ; i++) {
            auto current_byte = compressed_bytes[count_bytes + i];

            if (current_byte == 0 && false == isLastLevel(
                count_bytes + i, compressed_octree.num_of_leaves, compressed_bytes.size())) {
                // Lost Byte
                next_centers[count_next_centers] = centers[i];
                count_next_centers++;
            }

            for(int k = 0 ; k < 8 ; k++) {
                if(((current_byte >> k) & 1 )!= 0) {
                    Eigen::Vector4f child_center = getChildCenter(centers[i], current_side_length, k);
                    next_centers[count_next_centers] = child_center;
                    count_next_centers++;
                }
            }
        }
        count_bytes += count_centers;
        count_centers = count_next_centers;
        total_centers += count_next_centers;
        memcpy(centers, next_centers, count_next_centers * sizeof(Eigen::Vector4f));
        current_side_length = current_side_length / 2;
    }

    std::vector<Eigen::Vector4f> decompressed_centers;
    for (int i = 0; i < compressed_octree.num_of_leaves; i++) {
        auto curr_center = *(centers + i);
        decompressed_centers.push_back(curr_center);
    }

    return decompressed_centers;
}

std::vector<Color> decompressColors(std::vector<uint8_t> compressed_bytes) {
    std::vector<uint8_t> decoded_bytes;
    JpegDecoder* jpeg_decoder = new JpegDecoder();
    jpeg_decoder->decode(compressed_bytes, decoded_bytes);

    std::vector<Color> decompressed_colors;
    for (int i = 0; i < decoded_bytes.size()/3; i++) {
        Color c;
        c.r = decoded_bytes[3*i+0];
        c.g = decoded_bytes[3*i+1];
        c.b = decoded_bytes[3*i+2];
        decompressed_colors.push_back(c);
    }
    return decompressed_colors;
}