#include "common.h"

Eigen::Vector3f getRootCenter(OctreeType& octree) {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    Eigen::Vector3f root_center = Eigen::Vector3f((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);
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

void writeToFile(std::string file_name, std::vector<Eigen::Vector4f> centers) {
    std::ofstream outputFile(file_name);

    if (outputFile.is_open()) {
        outputFile << "ply" << "\n";
        outputFile << "format ascii 1.0" << "\n";
        outputFile << "element vertex " << centers.size() << "\n";
        outputFile << "property float x" << "\n";
        outputFile << "property float y" << "\n";
        outputFile << "property float z" << "\n";
        outputFile << "end_header" << "\n";

        for (const auto& center : centers) {
            outputFile << center.x() << " " << center.y() << " " << center.z() << "\n";
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
        std::cerr << "Wring drop_probability_percentage " << drop_probability_percentage << std::endl;
        exit(1);
    }

    double drop_probability = 100 * drop_probability_percentage;
    return getRandomNumber(0, 10000) <= drop_probability;
}