#ifndef COMMON_H
#define COMMON_H

#include <chrono>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <tuple>
#include <vector>

#include "colors.h"

#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

const float DROP_PROBABILITY = 0.1;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::octree::OctreePointCloud<PointType> OctreeType;
const int SAMPLING_FACTOR = 1;

struct negotiablePartOfCompressedOctree {
    std::vector<uint8_t> negotiable_bytes;
    uint64_t num_of_leaves;
};

struct nonNegotiablePartOfCompressedOctree {
    std::vector<uint8_t> non_negotiable_bytes;
    uint64_t num_of_negotiable_bytes;
    std::vector<float> root_center;
    float root_side_length;
};

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    Color(): r(0), g(0), b(0) {}
    Color(uint8_t r, uint8_t g, uint8_t b): r(r), g(g), b(b) {}
};

std::vector<float> getRootCenter(OctreeType& octree);
Eigen::Vector4f getChildCenter(Eigen::Vector4f parent_center, float side_len, uint8_t bit_pos);
void writeToFile(std::string filename, std::vector<Eigen::Vector4f> centers, std::vector<Color> colors, float drop_prob, bool block_drop);
std::vector<bool> generate_mask_for_block_drops(double drop_probability, size_t N);
int getRandomNumber(int x, int y);
int dropOrNot(double drop_probability_percentage);
std::unordered_map<int, long long int> getNodeCountsPerLevel(OctreeType& octree);
void showStats(nonNegotiablePartOfCompressedOctree non_negotiable_comp_part, negotiablePartOfCompressedOctree negotiable_comp_part, std::vector<uint8_t> compressed_colors, std::vector<std::chrono::high_resolution_clock::time_point> time_points);
void test();
#endif