#include <filesystem>
#include <sstream> 

#include "common.h"
#include "encode.h"
#include "decode.h"

#include <pcl/filters/random_sample.h>

const char* dataset = "ricardo.ply";

int main() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;

    Reader.read(std::string("./../assets/") + dataset, *cloud);

    std::vector<int> percentages = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10};

    // Loop over each percentage
    for (const int& percentage : percentages) {
        // Calculate the number of points to sample
        std::size_t sample_size = static_cast<std::size_t>(cloud->width * cloud->height * (percentage / 100.0));

        // Create the random sampling filter
        pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
        pcl::RandomSample<PointType> random_sample;
        random_sample.setInputCloud(cloud);
        random_sample.setSample(static_cast<unsigned int>(sample_size));  // Sample specified percentage of the points
        random_sample.filter(*cloud_filtered);

        std::cout << "PointCloud after filtering to " << percentage << "\% has: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

        // Create the output filename
        std::string output_filename = "./../sampled_assets/" + std::string(dataset) + "_" + std::to_string((100-percentage)) + "dropped.ply";

        // Save the filtered point cloud to a new PLY file
        pcl::io::savePLYFileASCII(output_filename, *cloud_filtered);
        std::cout << "Saved filtered point cloud to " << output_filename << std::endl;
    }

    return 0;


}