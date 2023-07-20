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

        tcp_connection_info info;
        socklen_t len = sizeof(info);
        if (getsockopt(tcp_client_socket, IPPROTO_TCP, TCP_CONNECTION_INFO, &info, &len) != 0) {
            perror("getsockopt failed\n");
            return;
        }

        std::cout << "tcpi_snd_cwnd: " << info.tcpi_snd_cwnd << std::endl;
        std::cout << "tcpi_rttcur: " << info.tcpi_rttcur << std::endl;
        std::cout << "tcpi_snd_wnd: " << info.tcpi_snd_wnd << std::endl;

        const int sending_rate = 1000; // bytes per second
        const int packet_size = 1500 - 8 - 20; // 1500 MTU, 8 UDP header, 20 IP header

        auto interval = std::chrono::nanoseconds(static_cast<long long>(1000000000.0 * packet_size / sending_rate));
        std::this_thread::sleep_for(interval);

        // To be continued!
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
