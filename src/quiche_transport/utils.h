#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>

void export_logs(
    const std::unordered_map<char, int>& bytes_per_frame,
    const std::unordered_map<char, int>& frame_end_time,
    const std::unordered_map<char, int>& frame_start_time,
    const char* logs_dir,
    const int time_offset) {

    // Construct file paths for saving logs
    std::string bytes_per_frame_file = std::string(logs_dir) + "/bytes_per_frame.csv";
    std::string frame_time_file = std::string(logs_dir) + "/frame_time.csv";
    std::string e2e_frame_time_file = std::string(logs_dir) + "/e2e_frame_time.csv";

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
        for (const auto& [k, v] : frame_end_time) {
            auto frame_start_it = frame_start_time.find(k);
            if (frame_start_it == frame_start_time.end()) {
                std::cerr << "Frame Start Time Not Found: " << k << std::endl;
                exit(1);
            }

            frame_time_out << k << "," << (1.0 * v - frame_start_it->second)/1000.0 << "\n";
        }
        frame_time_out.close();
    } else {
        // Handle error opening file
        std::cerr << "Error opening file: " << frame_time_file << std::endl;
    }

     // Export frame time data
    std::ofstream e2e_frame_time_out(e2e_frame_time_file);
    if (e2e_frame_time_out.is_open()) {
        for (const auto& [k, v] : frame_end_time) {

            e2e_frame_time_out << k << "," << (1.0 * v - time_offset)/1000.0 << "\n";
        }
        e2e_frame_time_out.close();
    } else {
        // Handle error opening file
        std::cerr << "Error opening file: " << e2e_frame_time_file << std::endl;
    }
}
