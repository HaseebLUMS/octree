#ifndef COMMON_H
#define COMMON_H

#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <tuple>
#include <vector>

#include "libs/JpegEncoder.hpp"
#include "libs/JpegDecoder.hpp"

#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::octree::OctreePointCloud<PointType> OctreeType;

struct compressedOctree {
    std::vector<uint8_t> bytes;
    uint64_t num_of_leaves;
    Eigen::Vector3f root_center;
    float root_side_length;
};

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    Color(): r(0), g(0), b(0) {}
    Color(uint8_t r, uint8_t g, uint8_t b): r(r), g(g), b(b) {}
};

Eigen::Vector3f getRootCenter(OctreeType& octree);
Eigen::Vector4f getChildCenter(Eigen::Vector4f parent_center, float side_len, uint8_t bit_pos);
void writeToFile(double lost_probability, std::vector<Eigen::Vector4f> centers, std::vector<Color> colors);
int getRandomNumber(int x, int y);
int dropOrNot(double drop_probability_percentage);
std::unordered_map<int, long long int> getNodeCountsPerLevel(OctreeType& octree);

// Compression
std::vector<uint8_t> compressColors(pcl::PointCloud<PointType>::Ptr& cloud, std::vector<int> points_order);
void jpegCompress(vector<uint8_t>& orig_colors, vector<uint8_t>& compressed_colors, JpegEncoder* jpeg_encoder);

// Decompression
std::vector<Color> decompressColors(std::vector<uint8_t> compressed_colors);
void jpegDecompress(std::vector<uint8_t> compressed_bytes, std::vector<uint8_t>& decoded_bytes);

#endif