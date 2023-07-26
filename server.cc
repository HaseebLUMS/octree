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

std::chrono::microseconds getInterPacketDuration(int tcp_client_socket, const int packet_size_in_bytes) {
    tcp_connection_info info;
    socklen_t len = sizeof(info);
    if (getsockopt(tcp_client_socket, IPPROTO_TCP, TCP_CONNECTION_INFO, &info, &len) != 0) {
        perror("getsockopt failed\n");
        exit(1);
    }

    const double rtt_ms = std::max((double)info.tcpi_srtt, 0.1);
    const double owd_ms = rtt_ms / 2; // rough
    const double owds_in_a_sec = 1000/owd_ms;
    const double sending_rate_byte_per_sec = info.tcpi_snd_cwnd * owds_in_a_sec;

    auto interval = std::chrono::microseconds(static_cast<long long>(1000000.0 * packet_size_in_bytes / sending_rate_byte_per_sec));
    return interval;
}

int sendUDPData(int udp_socket, struct sockaddr_in client_addr, std::chrono::microseconds duration, int total_data, char* data_buffer) {
    int total_sent = 0;
    while (total_sent < total_data) {
        int bytes_sent = sendto(udp_socket, data_buffer + total_sent, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
        if (bytes_sent == -1) {
            return 1;
        }
        total_sent += bytes_sent;
        std::this_thread::sleep_for(duration);
    }

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

    return 0;
}

void handleTCPConnection(int tcp_client_socket, int udp_socket, struct sockaddr_in client_addr) {
    client_addr.sin_port = htons(CLIENT_UDP_PORT);

    int tcp_data_size = DATA_SIZE;
    std::vector<char> tcp_data_buffer(tcp_data_size, 'A');

    int udp_data_size = 2*DATA_SIZE;
     std::vector<char> udp_data_buffer(udp_data_size, 'B');

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
            auto duration = getInterPacketDuration(tcp_client_socket, BUFFER_SIZE);
            res = sendUDPData(udp_socket, client_addr, duration, udp_data_size, udp_data_buffer.data());
        } else {
            used_scheme += "TCP";
            res = sendTCPData(tcp_client_socket, udp_data_size, udp_data_buffer.data());
        }
        if (res != 0) return;

        auto end = std::chrono::high_resolution_clock::now();
        auto elapased = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "The transport scheme " << used_scheme << " took " << elapased.count() << " milliseconds." << std::endl;
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

    sockaddr_in tcp_server_addr;
    tcp_server_addr.sin_family = AF_INET;
    tcp_server_addr.sin_port = htons(SERVER_TCP_PORT);
    inet_pton(AF_INET, SERVER_IP, &(tcp_server_addr.sin_addr));

    if (bind(tcp_socket, (struct sockaddr *)&tcp_server_addr, sizeof(tcp_server_addr)) == -1) {
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
