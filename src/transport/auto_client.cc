#include "client.hpp"

int main(int argc, char* argv[]) {
    int tcp_socket = 0, udp_socket = 0;
    auto res = initiliazeClient(argc, argv, tcp_socket, udp_socket);
    if (res) return res;

    std::string transport_scheme_tcp = "TCP+TCP";
    std::string transport_scheme_tcp_udp_part_1 = "TCP+UDP(TCP)";
    std::string transport_scheme_tcp_udp_part_2 = "TCP+UDP(UDP)";
    
    int count = 200;
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
            receiveData(tcp_socket, RELIABLE_DATA_SIZE, transport_scheme_tcp_udp_part_1, count);
            receiveData(udp_socket, UNRELIABLE_DATA_SIZE, transport_scheme_tcp_udp_part_2, count);
        } else {
            receiveData(tcp_socket, RELIABLE_DATA_SIZE+UNRELIABLE_DATA_SIZE, transport_scheme_tcp, count);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto elapased = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    }

    close(udp_socket);
    close(tcp_socket);

    return 0;
}
