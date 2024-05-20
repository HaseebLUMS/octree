#include "client.hpp"

int main(int argc, char* argv[]) {
    int tcp_socket = 0, udp_socket = 0;
    auto res = initiliazeClient(argc, argv, tcp_socket, udp_socket);
    if (res) return res;

    std::string transport_scheme_tcp = "TCP+TCP";
    std::string transport_scheme_tcp_udp_part_1 = "TCP+UDP(TCP)";
    std::string transport_scheme_tcp_udp_part_2 = "TCP+UDP(UDP)";

    while (true) {
        std::string message = "udp";
        // std::cout << "Enter a message to send (or 'exit' to quit): ";
        // std::getline(std::cin, message);

        if (message == "exit") {
            break;
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
        std::cout << "The transport took " << elapased.count() << " milliseconds." << std::endl;

        break;
    }

    close(udp_socket);
    close(tcp_socket);

    return 0;
}
