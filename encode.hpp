#include "common.h"

compressedOctree compressOctree(OctreeType& octree) {
    std::vector<std::vector<uint8_t>> compressed_bytes(octree.getTreeDepth() + 1);

    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();
    uint64_t num_of_leaves = 0;

    while (it != it_end) {
        auto current_level = it.getCurrentOctreeDepth();
        compressed_bytes.at(current_level).push_back(it.getNodeConfiguration());
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
    
    return result;
}