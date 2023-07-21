#include <iostream>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <thread>

const char* SERVER_TCP_IP = "127.0.0.1";
const int TCP_PORT = 9999;
const int UDP_PORT = 8888;

std::chrono::microseconds getInterPacketDuration(int tcp_client_socket) {
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
    const int packet_size_in_bytes = 1400; // conservative estimate

    auto interval = std::chrono::microseconds(static_cast<long long>(1000000.0 * packet_size_in_bytes / sending_rate_byte_per_sec));
    return interval;
}

void handleTCPConnection(int tcp_client_socket) {
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

        std::cout << "Received TCP message from client: " << buffer << std::endl;

        send(tcp_client_socket, buffer, strlen(buffer), 0);
        auto duration = getInterPacketDuration(tcp_client_socket);
        // use this duration to pace the UDP sends
    }

    close(tcp_client_socket);
}

int main() {
    int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket == -1) {
        std::cerr << "Error creating TCP socket." << std::endl;
        return 1;
    }

    sockaddr_in tcp_server_addr;
    tcp_server_addr.sin_family = AF_INET;
    tcp_server_addr.sin_port = htons(TCP_PORT);
    inet_pton(AF_INET, SERVER_TCP_IP, &(tcp_server_addr.sin_addr));

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

    std::cout << "Server is listening for TCP connections on port " << TCP_PORT << std::endl;

    while (true) {
        int tcp_client_socket = accept(tcp_socket, nullptr, nullptr);
        if (tcp_client_socket == -1) {
            std::cerr << "Error accepting TCP connection." << std::endl;
            continue;
        }

        handleTCPConnection(tcp_client_socket);
    }

    close(tcp_socket);

    return 0;
}
