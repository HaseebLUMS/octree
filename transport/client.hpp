#include <iostream>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "config.h"

void showStats(
    std::chrono::system_clock::time_point start, 
    std::chrono::system_clock::time_point end,
    int total_received_bytes) {
    auto elapased = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time: " << elapased.count() << " milliseconds." << std::endl;
    double mega_bits_recvd = (total_received_bytes*8)/(1024*1024);
    double seconds = (double)elapased.count()/1000;
    double bwd = mega_bits_recvd/seconds;
    std::cout << "Bandwidth: " << bwd << " Mbps." << std::endl;
    std::cout << "DID YOU REMEMBER TO INCREASE OS UDP BUFFER SIZE? (check readme.md)" << std::endl;
}

void showStatsForAnalysis( std::chrono::system_clock::time_point start, 
    std::chrono::system_clock::time_point end,
    int total_received, int total_data_to_receive, const std::string& transport_scheme) {
    
    double percentage_data_recvd = 100*(double)total_received/total_data_to_receive;
    auto time_milliseconds = (std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count();

    std::cout << transport_scheme << " " << time_milliseconds << " " << percentage_data_recvd << std::endl;
}

void receiveData(int socket, int total_data_to_receive, const std::string& transport_scheme) {
    auto start = std::chrono::high_resolution_clock::now();
    char data_buffer[BUFFER_SIZE_WITH_EXTRA_ROOM];

    int total_received = 0;
    while (total_received < total_data_to_receive) {
        int bytes_received = 0;
        bytes_received = recv(socket, data_buffer, BUFFER_SIZE_WITH_EXTRA_ROOM, 0);

        if (bytes_received <= 0) {
            break;
        }

        total_received += bytes_received;
    }
    auto end = std::chrono::high_resolution_clock::now();
    // showStats(start, end, total_received);
    showStatsForAnalysis(start, end, total_received, total_data_to_receive, transport_scheme);
    // std::cout << "Received " << (100*((double)total_received/total_data_to_receive)) << "\% of " << total_data_to_receive << std::endl;
}