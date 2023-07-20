#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

const char* SERVER_TCP_IP = "127.0.0.1";
const int TCP_PORT = 9999;
const int UDP_PORT = 8888;

int main() {
    int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket == -1) {
        std::cerr << "Error creating TCP socket." << std::endl;
        return 1;
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    inet_pton(AF_INET, SERVER_TCP_IP, &(server_addr.sin_addr));

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
    }

    return 0;
}
