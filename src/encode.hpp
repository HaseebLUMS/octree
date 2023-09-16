#include "common.h"

inline bool isPointCardinalityLevel(int current_level, int max_level, std::unordered_map<int, long long> node_counts_per_level) {
    return node_counts_per_level[current_level] == node_counts_per_level[max_level];
}

void get_leaves_indices(pcl::octree::OctreeNode* node, std::vector<int> &indices) {
    if(node->getNodeType() != pcl::octree::LEAF_NODE) {
        std::cerr << "The node in get_leaves_indices is not a lead node" << std::endl;
        exit(1);
    }

    const auto& leaf_node = static_cast<pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(node);
    const auto& container = leaf_node->getContainer();
    container.getPointIndices(indices);
}

void compressImage(std::vector<uint8_t>& orig_colors, std::vector<uint8_t>& compressed_colors, ColorEncDec* color_enc_dec) {
    int image_width = 512;
    int image_height = 512;

    if (orig_colors.size()/3 > 512*512 && orig_colors.size()/3 <= 1024*512) {
        image_width = 1024;
    } else if (orig_colors.size()/3 > 1024*512 && orig_colors.size()/3 <= 1024*1024) {
        image_width = 1024;
        image_height = 1024;
    } else if (orig_colors.size()/3 > 1024*1024 && orig_colors.size()/3 <= 1024*2048) {
        image_width = 2048;
        image_height = 1024;
    } else if (orig_colors.size()/3 > 1024*2048) {
        image_width = 2048;
        image_height = 2048;
    }
    std::cout << "H: " << image_height << " W: " << image_width << std::endl;

    color_enc_dec->encode(orig_colors, compressed_colors, image_width, image_height);
}

std::tuple<nonNegotiablePartOfCompressedOctree, negotiablePartOfCompressedOctree, std::vector<int>> compressOctree(OctreeType& octree) {
    const auto& tree_depth = octree.getTreeDepth();
    std::vector<std::vector<uint8_t>> compressed_bytes(tree_depth + 1);
    std::vector<int> points_order;

    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();

    while (it != it_end) {
        const auto& current_level = it.getCurrentOctreeDepth();
        const uint8_t& occupancy_byte = it.getNodeConfiguration();

        if (tree_depth == current_level) {
            std::vector<int> point_indices;
            get_leaves_indices(it.getCurrentOctreeNode(), point_indices);
            if (point_indices.size() != 1) {
                std::cerr << "[compressOctree] At the last level, there should be 1 point per node. But that's not the case!" << std::endl;
                exit(1);
            }
            points_order.push_back(point_indices[0]);
        }

        compressed_bytes[current_level].push_back(occupancy_byte);
        it++;
    }

    if (compressed_bytes.size() < 3) {
        std::cerr << "[compressOctree] Less than 3 layer? What are you even doing?" << std::endl;
        exit(1);
    }

    // Note: Last Layer is always 000...
    // So second last layer is negotiable.
    // In cases with crazy high resolution, we might see several levels that are negotiable.
    // Not handling that case right now.

    std::vector<uint8_t> non_negotiable_bytes;
    for (int i = 0; i <= compressed_bytes.size()-3; i++) {
        const auto& level = compressed_bytes[i];
        for (const auto& bytes: level) {
            non_negotiable_bytes.push_back(bytes);
        }
    }

    std::vector<uint8_t> negotiable_bytes;
    for (const auto& bytes: compressed_bytes.at(compressed_bytes.size()-2)) {
        negotiable_bytes.push_back(bytes);
    }

    nonNegotiablePartOfCompressedOctree non_neg_result = {
        .non_negotiable_bytes = non_negotiable_bytes,
        .num_of_negotiable_bytes = negotiable_bytes.size(),
        .root_center = getRootCenter(octree),
        .root_side_length = (float)sqrt(octree.getVoxelSquaredSideLen(0)),
    };

    negotiablePartOfCompressedOctree neg_result = {
        .negotiable_bytes = negotiable_bytes,
        .num_of_leaves = compressed_bytes.at(compressed_bytes.size()-1).size()
    };


    return std::make_tuple(non_neg_result, neg_result, points_order);
}

std::vector<uint8_t> compressColors(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<int> points_order, ColorEncDec* color_enc_dec) {
    std::vector<uint8_t> orig_colors;
    std::vector<uint8_t> compressed_colors;

    int i = 0;
    for (auto ind : points_order) {
        if (i == 0) {
            const auto& p = cloud->points.at(ind);
            orig_colors.push_back(p.r);
            orig_colors.push_back(p.g);
            orig_colors.push_back(p.b);
        }
        i = (i + 1) % SAMPLING_FACTOR;
    }

    compressImage(orig_colors, compressed_colors, color_enc_dec);
    return compressed_colors;
}

