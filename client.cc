#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "config.h"

void receiveData(int socket, int total_data_to_receive) {
    char data_buffer[BUFFER_SIZE];

    int total_received = 0;
    while (total_received < total_data_to_receive) {
        int bytes_received = recv(socket, data_buffer, BUFFER_SIZE, 0);
        if (bytes_received == -1) {
            std::cerr << "Receiving data failed. Received " <<  total_received << " out of " << total_data_to_receive << std::endl;
            return;
        } else if (bytes_received == 0) {
            break;
        }

        total_received += bytes_received;
    }

    std::cout << "Received " << total_received << " bytes." << std::endl;
}

int main(int argc, char* argv[]) {
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

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 100000;
    setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

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

        receiveData(tcp_socket, RELIABLE_DATA_SIZE);
        if (message == "udp") {
            receiveData(udp_socket, UNRELIABLE_DATA_SIZE);
        } else {
            receiveData(tcp_socket, UNRELIABLE_DATA_SIZE);
        }
    }

    close(udp_socket);
    close(tcp_socket);

    return 0;
}
