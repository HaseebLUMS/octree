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

void jpegCompress(vector<uint8_t>& orig_colors, vector<uint8_t>& compressed_colors, JpegEncoder* jpeg_encoder) {
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

    jpeg_encoder->encode(orig_colors, compressed_colors, image_width, image_height); 
}

std::tuple<compressedOctree, std::vector<int>> compressOctree(OctreeType& octree, double drop_probability) {
    const auto& tree_depth = octree.getTreeDepth();
    std::vector<std::vector<uint8_t>> compressed_bytes(tree_depth + 1);
    std::vector<int> points_order;

    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();
    uint64_t num_of_leaves = 0;

    while (it != it_end) {
        const auto& current_level = it.getCurrentOctreeDepth();
        const uint8_t& occupancy_byte = it.getNodeConfiguration();

        if (tree_depth == current_level) {
            num_of_leaves++;
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

    std::vector<uint8_t> flat_bytes;
    for (const auto& level : compressed_bytes) {
        for (const auto& bytes: level) {
            flat_bytes.push_back(bytes);
        }
    }

    compressedOctree result = {
        .bytes = flat_bytes,
        .root_center = getRootCenter(octree),
        .root_side_length = (float)sqrt(octree.getVoxelSquaredSideLen(0)),
        .num_of_leaves = num_of_leaves
    };
    
    return std::make_tuple(result, points_order);
}

std::vector<uint8_t> compressColors(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<int> points_order) {
    std::vector<uint8_t> orig_colors;
    std::vector<uint8_t> compressed_colors;

    for (auto ind : points_order) {
        const auto& p = cloud->points.at(ind);
        orig_colors.push_back(p.r);
        orig_colors.push_back(p.g);
        orig_colors.push_back(p.b);
    }

    JpegEncoder* jpeg_encoder = new JpegEncoder();
    jpegCompress(orig_colors, compressed_colors, jpeg_encoder);
    return compressed_colors;
}

