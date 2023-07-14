#include "common.h"

inline bool isPointCardinalityLevel(int current_level, int max_level, std::unordered_map<int, long long> node_counts_per_level) {
    return node_counts_per_level[current_level] == node_counts_per_level[max_level];
}

void get_leaves_indices(pcl::octree::OctreeNode* node, std::vector<int> &indices) {
    if(node->getNodeType() != pcl::octree::LEAF_NODE) {
        std::cerr << "The node in get_leaves_indices is not a lead node" << std::endl;
        exit(1);
    }

    pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>* leaf_node = static_cast<pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(node);
    pcl::octree::OctreeContainerPointIndices container = leaf_node->getContainer();
    container.getPointIndices(indices);
}


std::tuple<compressedOctree, std::vector<int>> compressOctree(OctreeType& octree, double drop_probability) {
    std::vector<std::vector<uint8_t>> compressed_bytes(octree.getTreeDepth() + 1);
    std::vector<int> points_order;
    int num_of_pclevels_bytes = 0;

    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();
    auto node_counts_per_level = getNodeCountsPerLevel(octree);
    uint64_t num_of_leaves = 0;
    int lost_bytes = 0;

    while (it != it_end) {
        auto current_level = it.getCurrentOctreeDepth();
        uint8_t occupancy_byte = it.getNodeConfiguration();
        
        // Simulating drops
        if (isPointCardinalityLevel(current_level, octree.getTreeDepth(), node_counts_per_level)) {
            num_of_pclevels_bytes++;
            if (true == dropOrNot(drop_probability)) {
                lost_bytes++;
                occupancy_byte = 0;
            }
        }

        // Getting points order (needed for ordering color bytes)
        if (octree.getTreeDepth() == current_level) {
            std::vector<int> point_indices;
            get_leaves_indices(it.getCurrentOctreeNode(), point_indices);
            if (point_indices.size() != 1) {
                std::cerr << "[compressOctree] At the last level, there should be 1 point per node. But that's not the case!" << std::endl;
                exit(1);
            }
            points_order.insert(points_order.end(), point_indices.begin(), point_indices.end());
        }

        compressed_bytes.at(current_level).push_back(occupancy_byte);
        if (current_level == octree.getTreeDepth()) {
            num_of_leaves++;
        }

        it++;
    }

    std::vector<uint8_t> flat_bytes;
    for (auto level : compressed_bytes) {
        for (auto bytes: level) {
            flat_bytes.push_back(bytes);
        }
    }

    compressedOctree result = {
        .bytes = flat_bytes,
        .root_center = getRootCenter(octree),
        .root_side_length = (float)sqrt(octree.getVoxelSquaredSideLen(0)),
        .num_of_leaves = num_of_leaves
    };
    
    std::cout << "Lost Bytes: " << lost_bytes << " / " << flat_bytes.size() << " (" << (100*((double)lost_bytes/flat_bytes.size())) << "), # of PClevels bytes = " << num_of_pclevels_bytes << std::endl;
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

void jpegCompress(vector<uint8_t>& orig_colors, vector<uint8_t>& compressed_colors, JpegEncoder* jpeg_encoder) {
    int color_size = orig_colors.size();
    int image_width = 1024;
    int image_height = 1024;

    // if(orig_colors.size()/3 < 1024*512) {
    //     image_height = 512;
    // } else if(orig_colors.size() / 3 >= 1024 * 1024) {
    //     image_width = 2048;
    // }

    jpeg_encoder->encode(orig_colors, compressed_colors, image_width, image_height); 
}
