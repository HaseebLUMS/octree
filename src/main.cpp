#include <filesystem>
#include <sstream> 
#include <sys/times.h>
#include <ctime>

#include "common.h"
#include "encode.h"
#include "decode.h"

const char* dataset = "soldier.ply";

// Get CPU times
void get_cpu_times(clock_t &utime, clock_t &stime) {
    struct tms time_sample;
    times(&time_sample);
    utime = time_sample.tms_utime;
    stime = time_sample.tms_stime;
}

// Calculate CPU load
double calculate_cpu_load(clock_t start_utime, clock_t start_stime, clock_t end_utime, clock_t end_stime, clock_t clock_ticks) {
    clock_t user_time_diff = end_utime - start_utime;
    clock_t sys_time_diff = end_stime - start_stime;
    return (double)(user_time_diff + sys_time_diff) / clock_ticks;
}

int main() {
    clock_t clock_ticks = sysconf(_SC_CLK_TCK);
    clock_t start_utime, start_stime;
    clock_t end_utime, end_stime;

    std::vector<int> percentages = {100, 90, 80, 70, 60, 50};  // for sampling clouds, elem shows result size
    // std::vector<int> percentages = {100};
    std::vector<double> voxel_sizes = {1};

    pcl::PointCloud<PointType>::Ptr parent_cloud (new pcl::PointCloud<PointType>);
    pcl::PLYReader Reader;
    Reader.read(std::string("./../assets/") + dataset, *parent_cloud);

    uint64_t data_size = 0;

    get_cpu_times(start_utime, start_stime);

    for (const int& percentage : percentages) {
        std::size_t sample_size = static_cast<std::size_t>(parent_cloud->width * parent_cloud->height * (percentage / 100.0));

        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::RandomSample<PointType> random_sample;
        random_sample.setInputCloud(parent_cloud);
        random_sample.setSample(static_cast<unsigned int>(sample_size));  // Sample specified percentage of the points
        random_sample.filter(*cloud);

        for (auto vox : voxel_sizes) {
            OctreeType octree(vox);
            octree.setInputCloud(cloud);
            octree.defineBoundingBox();
            octree.addPointsFromInputCloud();

            // Compression
            auto t1 = std::chrono::high_resolution_clock::now();
            WebpEncDec* color_enc = new WebpEncDec();
            // JpegEncDec* color_enc = new JpegEncDec();
            auto [non_negotiable_comp_part, negotiable_comp_part, points_order] = compressOctree(octree);
            auto compressed_colors = compressColors(cloud, points_order, color_enc);
            auto t2 = std::chrono::high_resolution_clock::now();

            data_size += sizeof(non_negotiable_comp_part) + sizeof(uint8_t)*non_negotiable_comp_part.non_negotiable_bytes.size();
            data_size += sizeof(float)*non_negotiable_comp_part.root_center.size();

            data_size += sizeof(negotiable_comp_part) + sizeof(uint8_t)*negotiable_comp_part.negotiable_bytes.size();
            
            data_size += sizeof(uint8_t)*compressed_colors.size();

            continue;  // don't want to decompression right now

            /**
             * Following items need to be sent to the receiver so that a volumetric frame may be constructed:
             * non_negotiable_comp_part, negotiable_comp_part, compressed_colors
            */

            // Decompression
            auto t3 = std::chrono::high_resolution_clock::now();
            WebpEncDec* color_dec = new WebpEncDec();
            // JpegEncDec* color_dec = new JpegEncDec();
            auto [decompressed_centers, side_length] = decompressNonNegotiableBytes(non_negotiable_comp_part);
            auto points = decompressNegotiableBytes(negotiable_comp_part, decompressed_centers, side_length);
            auto decompressed_colors = decompressColors(compressed_colors, color_dec);
            auto t4 = std::chrono::high_resolution_clock::now();

            // std::vector<double> drops = {0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 0.6, 0.7};
            std::vector<double> drops = {0.0};

            std::string path = std::string("./../output/") + dataset + std::string("/");
            if (!std::filesystem::exists(path)) {
                if (std::filesystem::create_directory(path)) {
                    std::cout << "Directory created successfully: " << path << std::endl;
                } else {
                    std::cerr << "Failed to create directory: " << path << std::endl;
                }
            }

            for (auto d : drops) {
                std::ostringstream s;
                s << path << d;
                std::string str = s.str(); 

                writeToFile(str + std::string(".ply"), points, decompressed_colors, d, false);
                // writeToFile(str + std::string("_block.ply"), points, decompressed_colors, d, true);
            }

            showStats(non_negotiable_comp_part, negotiable_comp_part, compressed_colors, {t1, t2, t3, t4});
            test();
        }

        get_cpu_times(end_utime, end_stime);

        double cpu_load = calculate_cpu_load(start_utime, start_stime, end_utime, end_stime, clock_ticks);
        std::cout << percentage << " ] CPU Load: " << cpu_load * 100 << "%" << std::endl;
        std::cout << percentage << " ] Storage: " <<  (1.0L * data_size / (1024*1024)) << "MB" << std::endl;
    }

    get_cpu_times(end_utime, end_stime);

    double cpu_load = calculate_cpu_load(start_utime, start_stime, end_utime, end_stime, clock_ticks);
    std::cout << "CPU Load: " << cpu_load * 100 << "%" << std::endl;

    std::cout << "Total bytes to transmit: " << (1.0L * data_size / (1024*1024)) << " MB" << std::endl;
}