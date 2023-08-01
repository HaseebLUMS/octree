#include "client.hpp"

int main(int argc, char* argv[]) {
    int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
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

    struct timeval timeout_udp;
    timeout_udp.tv_sec = 0;
    timeout_udp.tv_usec = UDP_SOCKET_WAIT;
    setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout_udp, sizeof(timeout_udp));

    if (connect(tcp_socket, (struct sockaddr *)&server_tcp_addr, sizeof(server_tcp_addr)) == -1) {
        std::cerr << "Error connecting to TCP server." << std::endl;
        close(tcp_socket);
        return 1;
    }

    std::string transport_scheme_tcp = "TCP";
    std::string transport_scheme_tcp_udp_part_1 = "TCP+UDP(TCP)";
    std::string transport_scheme_tcp_udp_part_2 = "TCP+UDP(UDP)";
    
    int count = 100;
    while (count--) {
        std::string message;
        if (count%2) {
            message = "tcp";
        } else {
            message = "udp";
        }

        auto start = std::chrono::high_resolution_clock::now();
        send(tcp_socket, message.c_str(), message.size(), 0);

        if (message == "udp") {
            receiveData(tcp_socket, RELIABLE_DATA_SIZE, transport_scheme_tcp_udp_part_1);
            receiveData(udp_socket, UNRELIABLE_DATA_SIZE, transport_scheme_tcp_udp_part_2);
        } else {
            receiveData(tcp_socket, RELIABLE_DATA_SIZE+UNRELIABLE_DATA_SIZE, transport_scheme_tcp);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto elapased = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    }

    close(udp_socket);
    close(tcp_socket);

    return 0;
}
