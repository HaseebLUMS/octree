#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>

void export_logs(
    const std::unordered_map<char, int>& bytes_per_frame,
    const std::unordered_map<char, int>& frame_time,
    const char* logs_dir,
    const int time_offset) {

    // Construct file paths for saving logs
    std::string bytes_per_frame_file = std::string(logs_dir) + "/bytes_per_frame.csv";
    std::string frame_time_file = std::string(logs_dir) + "/frame_time.csv";

    // Export bytes per frame data
    std::ofstream bytes_per_frame_out(bytes_per_frame_file);
    if (bytes_per_frame_out.is_open()) {
        for (const auto& entry : bytes_per_frame) {
            bytes_per_frame_out << entry.first << "," << entry.second << "\n";
        }
        bytes_per_frame_out.close();
    } else {
        // Handle error opening file
        std::cerr << "Error opening file: " << bytes_per_frame_file << std::endl;
    }

    // Export frame time data
    std::ofstream frame_time_out(frame_time_file);
    if (frame_time_out.is_open()) {
        for (const auto& entry : frame_time) {
            frame_time_out << entry.first << "," << (1.0 * entry.second - time_offset)/1000.0 << "\n";
        }
        frame_time_out.close();
    } else {
        // Handle error opening file
        std::cerr << "Error opening file: " << frame_time_file << std::endl;
    }
}
