#ifndef COMMON_H
#define COMMON_H

#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <vector>

#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::octree::OctreePointCloud<PointType> OctreeType;

struct compressedOctree {
    std::vector<uint8_t> bytes;
    uint64_t num_of_leaves;
    Eigen::Vector3f root_center;
    float root_side_length;
};

Eigen::Vector3f getRootCenter(OctreeType& octree);
Eigen::Vector4f getChildCenter(Eigen::Vector4f parent_center, float side_len, uint8_t bit_pos);
void writeToFile(double lost_probability, std::vector<Eigen::Vector4f> centers);
int getRandomNumber(int x, int y);
int dropOrNot(double drop_probability_percentage);
std::unordered_map<int, long long int> getNodeCountsPerLevel(OctreeType& octree);

#endif