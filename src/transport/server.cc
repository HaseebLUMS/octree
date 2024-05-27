#include <iostream>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <thread>
#include <vector>

#include "config.h"

std::chrono::microseconds getInterPacketDuration(const int packet_size_in_bytes, const int packet_rate_in_Mbps) {
    const int packet_size_in_bits = packet_size_in_bytes * 8;
    auto interval = std::chrono::microseconds(static_cast<long long>((1000000.0 * packet_size_in_bits / (packet_rate_in_Mbps*1000000)) - CUSHION_IN_SLEEP_CALCULATIONS));
    std::cout << "Sleeping duration: " << interval.count() << " microseconds" << std::endl;
    return interval;
}

int sendUDPData(int udp_socket, struct sockaddr_in client_addr, std::chrono::microseconds duration, int total_data, char* data_buffer) {
    int total_sent = 0;
    std::chrono::microseconds total_sleep;
    while (total_sent < total_data) {
        auto start = std::chrono::high_resolution_clock::now();
        int bytes_sent = sendto(udp_socket, data_buffer + total_sent, std::min(BUFFER_SIZE, total_data-total_sent), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));

        if (bytes_sent == -1) {
            return 1;
        }
        total_sent += bytes_sent;
        auto end = std::chrono::high_resolution_clock::now();
        auto elapased = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
        total_sleep = duration-elapased;
        std::this_thread::sleep_for(total_sleep);
    }
    // std::cout << "Bytes sent: " << total_sent  << " with total sleep " << total_sleep.count() << std::endl;

    return 0;
}

int sendTCPData(int tcp_client_socket, int total_data, char* data_buffer) {
    int total_sent = 0;
    while (total_sent < total_data) {
        int bytes_sent = send(tcp_client_socket, data_buffer + total_sent, std::min(BUFFER_SIZE, total_data-total_sent), 0);
        if (bytes_sent == -1) {
            std::cerr << "Sending data failed." << std::endl;
            return 1;
        }
        total_sent += bytes_sent;
    }
    // std::cout << "Bytes sent: " << total_sent << std::endl;

    return 0;
}

void handleTCPConnection(int tcp_client_socket, int udp_socket, struct sockaddr_in client_addr) {
    client_addr.sin_port = htons(CLIENT_UDP_PORT);

    int tcp_data_size = RELIABLE_DATA_SIZE;
    std::vector<char> tcp_data_buffer(tcp_data_size, 'A');

    int udp_data_size = UNRELIABLE_DATA_SIZE;
    std::vector<char> udp_data_buffer(udp_data_size, 'B');
    udp_data_buffer[udp_data_size-1] = 'C';
    
    auto duration = getInterPacketDuration(BUFFER_SIZE, PACKET_RATE_Mbps);
    
    while (true) {
        char buffer[1024] = {0};
        int bytes_received = recv(tcp_client_socket, buffer, sizeof(buffer), 0);

        if (bytes_received == -1) {
            std::cerr << "Error receiving TCP message from client." << std::endl;
            break;
        } else if (bytes_received == 0) {
            std::cout << "Client disconnected." << std::endl;
            break;
        }
        auto start = std::chrono::high_resolution_clock::now();
        
        std::string used_scheme = "TCP+";
        auto res = sendTCPData(tcp_client_socket, tcp_data_size, tcp_data_buffer.data());
        if (res != 0) return;
        
        if (strncmp(buffer, "udp", 3) == 0) {
            used_scheme += "UDP";
            
            res = sendUDPData(udp_socket, client_addr, duration, udp_data_size, udp_data_buffer.data());
        } else {
            used_scheme += "TCP";
            res = sendTCPData(tcp_client_socket, udp_data_size, udp_data_buffer.data());
        }
        if (res != 0) return;

        auto end = std::chrono::high_resolution_clock::now();
        auto elapased = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // std::cout << "The transport scheme " << used_scheme << " took " << elapased.count() << " milliseconds." << std::endl;
    }

    close(tcp_client_socket);
}

int main(int argc, char* argv[]) {
    // Setup TCP socket
    int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket == -1) {
        std::cerr << "Error creating TCP socket." << std::endl;
        return 1;
    }

    sockaddr_in server_tcp_addr;
    server_tcp_addr.sin_family = AF_INET;
    server_tcp_addr.sin_port = htons(SERVER_TCP_PORT);
    inet_pton(AF_INET, SERVER_IP, &(server_tcp_addr.sin_addr));

    if (bind(tcp_socket, (struct sockaddr *)&server_tcp_addr, sizeof(server_tcp_addr)) == -1) {
        std::cerr << "Error binding TCP socket." << std::endl;
        close(tcp_socket);
        return 1;
    }

    if (listen(tcp_socket, 5) == -1) {
        std::cerr << "Error listening on TCP socket." << std::endl;
        close(tcp_socket);
        return 1;
    }

    std::cout << "Server is listening for TCP connections on port " << SERVER_TCP_PORT << std::endl;

    // Setup UDP socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        std::cerr << "Error creating UDP socket." << std::endl;
        return 1;
    }

    struct sockaddr_in server_udp_addr;
    std::memset(&server_udp_addr, 0, sizeof(server_udp_addr));
    server_udp_addr.sin_family = AF_INET;
    server_udp_addr.sin_port = htons(SERVER_UDP_PORT); // Replace 12345 with the port you want to listen on
    inet_pton(AF_INET, SERVER_IP, &(server_udp_addr.sin_addr));

    if (bind(udp_socket, (struct sockaddr*)&server_udp_addr, sizeof(server_udp_addr)) < 0) {
        std::cerr << "Error binding UDP socket." << std::endl;
        close(udp_socket);
        return 1;
    }

    std::vector<std::thread> conn_threads;
    while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int tcp_client_socket = accept(tcp_socket, (struct sockaddr*)&client_addr, &client_addr_len);
        if (tcp_client_socket == -1) {
            std::cerr << "Error accepting TCP connection." << std::endl;
            continue;
        }

        conn_threads.emplace_back(handleTCPConnection, tcp_client_socket, udp_socket, client_addr);
    }

    for (auto& thread : conn_threads) {
        thread.join();
    }

    close(tcp_socket);
    close(udp_socket);
    return 0;
}
