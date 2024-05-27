#include "common.h"

bool isLastLevel(int curr_node_index, int nodes_in_last_level, int total_nodes) {
    int starting_index_of_last_level = total_nodes - nodes_in_last_level;
    return curr_node_index >= starting_index_of_last_level;
}

std::tuple<std::vector<Eigen::Vector4f>, float> decompressNonNegotiableBytes(nonNegotiablePartOfCompressedOctree compressed_octree) {
    auto compressed_bytes = compressed_octree.non_negotiable_bytes;

    Eigen::Vector4f* centers = (Eigen::Vector4f*) malloc(
        compressed_octree.num_of_negotiable_bytes*sizeof(Eigen::Vector4f));

    Eigen::Vector4f* next_centers = (Eigen::Vector4f*) malloc(
        compressed_octree.num_of_negotiable_bytes*sizeof(Eigen::Vector4f));

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
    for (int i = 0; i < compressed_octree.num_of_negotiable_bytes; i++) {
        auto curr_center = *(centers + i);
        decompressed_centers.push_back(curr_center);
    }

    return make_tuple(decompressed_centers, current_side_length);
}

// std::vector<Eigen::Vector4f> simulate_drops(float drop_percentage, const std::vector<Eigen::Vector4f>& points) {
//     // Seed the random number generator
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<float> dis(0.0f, 1.0f);

//     std::vector<Eigen::Vector4f> remaining_points;
//     for (const auto& point : points) {
//         // Generate a random number between 0 and 1
//         float rand_num = dis(gen);
//         // If the random number is less than the drop percentage, drop the point
//         if (rand_num < drop_percentage) {
//             // Do not add the point to the dropped_points vector
//             continue;
//         }
//         // Add the point to the dropped_points vector
//         remaining_points.push_back(point);
//     }
//     return remaining_points;
// }


// TODO: A lot of common code in decompressNonNegotiableBytes and decompressNegotiableBytes. Factor that out.
std::vector<Eigen::Vector4f> decompressNegotiableBytes(negotiablePartOfCompressedOctree compressed_octree, std::vector<Eigen::Vector4f> decompressed_centers, float side_length) {
    auto compressed_bytes = compressed_octree.negotiable_bytes;
    
    Eigen::Vector4f* centers = (Eigen::Vector4f*) malloc(
        compressed_octree.num_of_leaves*sizeof(Eigen::Vector4f));
    
    for (int i = 0; i < decompressed_centers.size(); i++) {
        centers[i] = decompressed_centers[i];
    }

    Eigen::Vector4f* next_centers = (Eigen::Vector4f*) malloc(
        compressed_octree.num_of_leaves*sizeof(Eigen::Vector4f));

    int count_bytes = 0;
    int count_centers = decompressed_centers.size();
    auto current_side_length = side_length;

    while(count_bytes < compressed_bytes.size()) {
        int count_next_centers = 0;

        for(int i = 0; i < count_centers ; i++) {
            auto current_byte = compressed_bytes[count_bytes + i];

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
        memcpy(centers, next_centers, count_next_centers * sizeof(Eigen::Vector4f));
        current_side_length = current_side_length / 2;
    }

    std::vector<Eigen::Vector4f> points;
    for (int i = 0; i < compressed_octree.num_of_leaves; i++) {
        auto curr_center = *(next_centers + i);
        points.push_back(curr_center);
    }


    // std::cout << "Original Points: " << points.size() << std::endl;

    /**
     * Used for achieving the effect of our Octavious system. 
    */
    // points = simulate_drops(DROP_PROBABILITY, points);

    // std::cout << "Remaining Points: " << points.size() << std::endl;
    return points;
}

std::vector<Color> expandColors(std::vector<Color> decompressed_colors) {
    std::vector<Color> expanded;
    for (auto ele: decompressed_colors) {
        for (int i = 0; i < SAMPLING_FACTOR; i++) {
            expanded.push_back(ele);

        }
    }
    return expanded;
}

std::vector<Color> decompressColors(std::vector<uint8_t> compressed_bytes, ColorEncDec* color_enc_dec) {
    std::vector<uint8_t> decoded_bytes;
    color_enc_dec->decode(compressed_bytes, decoded_bytes);

    std::vector<Color> decompressed_colors;
    for (int i = 0; i < decoded_bytes.size()/3; i++) {
        Color c;
        c.r = decoded_bytes[3*i+0];
        c.g = decoded_bytes[3*i+1];
        c.b = decoded_bytes[3*i+2];
        decompressed_colors.push_back(c);
    }

    if (SAMPLING_FACTOR > 1) {
        auto res = expandColors(decompressed_colors);
        return res;
    }
    return decompressed_colors;
}