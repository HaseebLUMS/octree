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
    int total_received, int total_data_to_receive, const std::string& transport_scheme, int message_id) {
    
    double percentage_data_recvd = 100*(double)total_received/total_data_to_receive;
    auto time_milliseconds = (std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count();

    std::cout << message_id << " " << transport_scheme << " " << time_milliseconds << " " << percentage_data_recvd << std::endl;
}

void receiveData(int socket, int total_data_to_receive, const std::string& transport_scheme, int message_id=-1) {
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
    showStatsForAnalysis(start, end, total_received, total_data_to_receive, transport_scheme, message_id);
    // std::cout << "Received " << (100*((double)total_received/total_data_to_receive)) << "\% of " << total_data_to_receive << std::endl;
}

int initiliazeClient(int argc, char* argv[], int& tcp_socket, int& udp_socket) {
    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket == -1) {
        std::cerr << "Error creating TCP socket." << std::endl;
        return 1;
    }

    int client_tcp_port = CLIENT_TCP_PORT;
    if (argc >= 2) client_tcp_port = atoi(argv[1]);

    sockaddr_in server_tcp_addr;
    server_tcp_addr.sin_family = AF_INET;
    server_tcp_addr.sin_port = htons(SERVER_TCP_PORT);
    inet_pton(AF_INET, SERVER_IP, &(server_tcp_addr.sin_addr));

    struct sockaddr_in client_tcp_addr;
    memset(&client_tcp_addr, 0, sizeof(client_tcp_addr));
    client_tcp_addr.sin_family = AF_INET;
    client_tcp_addr.sin_port = htons(client_tcp_port);
    client_tcp_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(tcp_socket, (struct sockaddr*)&client_tcp_addr, sizeof(client_tcp_addr)) == -1) {
        perror("Error binding TCP Socket");
        close(tcp_socket);
        exit(EXIT_FAILURE);
    }

    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        std::cerr << "Error creating UDP socket." << std::endl;
        return 1;
    }

    struct sockaddr_in client_udp_addr;
    std::memset(&client_udp_addr, 0, sizeof(client_udp_addr));
    client_udp_addr.sin_family = AF_INET;
    client_udp_addr.sin_port = htons(CLIENT_UDP_PORT); // Replace 12345 with the port you want to listen on
    inet_pton(AF_INET, CLIENT_IP, &(client_udp_addr.sin_addr));

    if (bind(udp_socket, (struct sockaddr*)&client_udp_addr, sizeof(client_udp_addr)) < 0) {
        std::cerr << "Error binding UDP socket." << std::endl;
        close(udp_socket);
        return 1;
    }

    struct timeval timeout_udp;
    timeout_udp.tv_sec = 0;
    timeout_udp.tv_usec = UDP_SOCKET_WAIT;
    setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout_udp, sizeof(timeout_udp));

    if (connect(tcp_socket, (struct sockaddr *)&server_tcp_addr, sizeof(server_tcp_addr)) == -1) {
        std::cerr << "Error connecting to TCP server." << std::endl;
        close(tcp_socket);
        return 1;
    }

    return 0;
}