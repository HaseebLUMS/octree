#include "common.h"

Eigen::Vector3f getRootCenter(OctreeType& octree) {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    Eigen::Vector3f root_center = Eigen::Vector3f((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);
    return root_center;
}