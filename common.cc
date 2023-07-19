#include "common.h"

std::vector<float> getRootCenter(OctreeType& octree) {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    std::vector<float> root_center;
    root_center.push_back((min_x+max_x)/2);
    root_center.push_back((min_y+max_y)/2);
    root_center.push_back((min_z+max_z)/2);
    return root_center;
}

Eigen::Vector4f getChildCenter(Eigen::Vector4f parent_center, float side_len, uint8_t bit_pos) {
    float margin = side_len / 4;
    Eigen::Vector4f child_center = parent_center;
    switch(bit_pos)
    {
        case 0:
            child_center[0] -= margin;
            child_center[1] -= margin;
            child_center[2] -= margin;
            break;
        case 1:
            child_center[0] -= margin;
            child_center[1] -= margin;
            child_center[2] += margin;
            break;
        case 2:
            child_center[0] -= margin;
            child_center[1] += margin;
            child_center[2] -= margin;
            break;
        case 3:
            child_center[0] -= margin;
            child_center[1] += margin;
            child_center[2] += margin;
            break;
        case 4:
            child_center[0] += margin;
            child_center[1] -= margin;
            child_center[2] -= margin;
            break;
        case 5:
            child_center[0] += margin;
            child_center[1] -= margin;
            child_center[2] += margin;
            break;
        case 6:
            child_center[0] += margin;
            child_center[1] += margin;
            child_center[2] -= margin;
            break;
        case 7:
            child_center[0] += margin;
            child_center[1] += margin;
            child_center[2] += margin;
            break;
        default:

            break;

    }
    return child_center;
}

// TODO: Get lost_probability out of here, not being used.
void writeToFile(double lost_probability, std::vector<Eigen::Vector4f> centers, std::vector<Color> colors) {
    
    std::string lp_str = std::to_string(lost_probability);
    lp_str.erase(lp_str.find_last_not_of('0') + 1, std::string::npos);
    std::string filename = "./output/output_lost_" + lp_str + ".ply";
    
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {
        outputFile << "ply" << "\n";
        outputFile << "format ascii 1.0" << "\n";
        outputFile << "element vertex " << centers.size() << "\n";
        outputFile << "property float x" << "\n";
        outputFile << "property float y" << "\n";
        outputFile << "property float z" << "\n";
        outputFile << "property uchar red" << "\n";
        outputFile << "property uchar green" << "\n";
        outputFile << "property uchar blue" << "\n";
        outputFile << "end_header" << "\n";
        for (int i = 0; i < centers.size(); i++) {
            const auto& center = centers[i];
            const auto& color = colors[i];
            // auto color = cloud->points[i];
            outputFile << center.x() << " " << center.y() << " " << center.z() << " " << (int)color.r << " " << (int)color.g << " " << (int)color.b  << "\n";
        }
        
        outputFile.close();
    } else {
        std::cerr << "Unable to open the file." << std::endl;
    }
}

int getRandomNumber(int x, int y) {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<int> distribution(x, y);
    return distribution(generator);
}

int dropOrNot(double drop_probability_percentage) {
    if (drop_probability_percentage == 0) {
        return false;
    }

    if (drop_probability_percentage < 0.01 || drop_probability_percentage > 100) {
        std::cerr << "Wrong drop_probability_percentage " << drop_probability_percentage << std::endl;
        exit(1);
    }

    double drop_probability = 100 * drop_probability_percentage;
    return getRandomNumber(0, 10000) <= drop_probability;
}

std::unordered_map<int, long long int> getNodeCountsPerLevel(OctreeType& octree) {
    auto it = octree.depth_begin();
    auto it_end = octree.depth_end();

    std::unordered_map<int, long long int> node_counts_per_level;
    while (it != it_end) {
        node_counts_per_level[it.getCurrentOctreeDepth()]++;
        it++;
    }
    return node_counts_per_level;
}

void showStats(nonNegotiablePartOfCompressedOctree non_negotiable_comp_part, negotiablePartOfCompressedOctree negotiable_comp_part, std::vector<uint8_t> compressed_colors) {
    double total_bytes = negotiable_comp_part.negotiable_bytes.size() + non_negotiable_comp_part.non_negotiable_bytes.size();
    std::cout << "Non Negotiable Bytes: " << non_negotiable_comp_part.non_negotiable_bytes.size() << std::endl;
    std::cout << "Negotiable Bytes: " << negotiable_comp_part.negotiable_bytes.size() << std::endl;
    std::cout << "Total Geometry Bytes: " << total_bytes << std::endl;
    std::cout << "Total Color Bytes: " << compressed_colors.size() << std::endl;
    std::cout << "Negotiable share (among geometry): " << (100.0*negotiable_comp_part.negotiable_bytes.size())/total_bytes << std::endl;
}