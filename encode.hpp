#include "common.h"

std::vector<std::vector<uint8_t>> compressOctree(OctreeType& octree) {
    std::vector<std::vector<uint8_t>> result(octree.getTreeDepth() + 1);

    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();

    while (it != it_end) {
        auto current_level = it.getCurrentOctreeDepth();
        result.at(current_level).push_back(it.getNodeConfiguration());
        it++;
    }
    
    return result;
}