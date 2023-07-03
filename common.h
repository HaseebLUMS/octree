#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <queue>
#include <vector>

#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::octree::OctreePointCloud<PointType> OctreeType;

Eigen::Vector3f getRootCenter(OctreeType& octree);

#endif