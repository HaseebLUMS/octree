#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "config.h"

int main() {
    int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket == -1) {
        std::cerr << "Error creating TCP socket." << std::endl;
        return 1;
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(CLIENT_TCP_PORT);
    inet_pton(AF_INET, CLIENT_IP, &(server_addr.sin_addr));

    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
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

    if (connect(tcp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        std::cerr << "Error connecting to TCP server." << std::endl;
        close(tcp_socket);
        return 1;
    }

    while (true) {
        std::string message;
        std::cout << "Enter a message to send (or 'exit' to quit): ";
        std::getline(std::cin, message);

        if (message == "exit") {
            break;
        }

        send(tcp_socket, message.c_str(), message.size(), 0);

        char buffer[1024] = {0};
        int bytes_received = recv(tcp_socket, buffer, sizeof(buffer), 0);
        if (bytes_received == -1) {
            std::cerr << "Error receiving TCP response from server." << std::endl;
        } else {
            std::cout << "Received TCP response from server: " << buffer << std::endl;
        }

        bytes_received = recv(udp_socket, buffer, sizeof(buffer), 0);
        if (bytes_received == -1) {
            std::cerr << "Error receiving UDP messgage from server." << std::endl;
        } else {
            std::cout << "Received UDP messgage from server: " << buffer << std::endl;
        }
    }

    close(udp_socket);
    close(tcp_socket);

    return 0;
}
