#include "common.h"

compressedOctree compressOctree(OctreeType& octree, double drop_probability) {
    std::vector<std::vector<uint8_t>> compressed_bytes(octree.getTreeDepth() + 1);

    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();
    uint64_t num_of_leaves = 0;
    int lost_bytes = 0;

    while (it != it_end) {
        auto current_level = it.getCurrentOctreeDepth();
        uint8_t occupancy_byte = it.getNodeConfiguration();
        
        if (current_level == octree.getTreeDepth() && true == dropOrNot(drop_probability)) {
            lost_bytes++;
            occupancy_byte = 0;
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
    
    std::cout << "Lost Bytes: " << lost_bytes << std::endl;
    return result;
}