#include <cassert>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <ctime>
#include <thread>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <linux/net_tstamp.h>
#include <linux/ip.h>
#include <ev.h>
#include <uthash.h>
#include <quiche.h>

#define LOCAL_CONN_ID_LEN 16
#define MAX_DATAGRAM_SIZE 1350
#define MAX_PKT_SIZE 1300 // Used for sending unreliable datagrams

constexpr uint64_t NANOS_PER_SEC = 1'000'000'000;

struct connections {
    int sock;

    struct sockaddr *local_addr;
    socklen_t local_addr_len;

    struct conn_io *h;
};

struct conn_io {
    ev_timer timer;
    int sock;
    uint8_t cid[LOCAL_CONN_ID_LEN];
    quiche_conn *conn;
    struct sockaddr_storage peer_addr;
    socklen_t peer_addr_len;
    UT_hash_handle hh;
};

class server {
public:
    std::string address;
    int port;
    connections *conns;

    server(std::string address, int port): address(address), port(port) {}

    int send_frame();
    void start_server();
    void serve_client();

private:
    void configure_quiche_server();
    void setsockopt_txtime(int sock);
};

int server::send_frame() {
    // serialize the reliable and unreliable parts
    // create necessary metadata (size of unreliable parts ?)
    // enqueue the data in all the client queues
}

void server::start_server() {
    // configure the necessary ev loops
    // spin thread `configure_quiche_server`

    // accept new connections
    // for each new connections, spin a thread `serve_client`, make a new client queue
}

void server::serve_client() {
    // keep watching the respective queue for new data
    // upon new data, send it to the client using quiche
}

void server::setsockopt_txtime(int sock) {
    struct sock_txtime so_txtime_val = { .clockid = CLOCK_MONOTONIC, .flags = 0 };
    so_txtime_val.flags = (SOF_TXTIME_REPORT_ERRORS);

    if (setsockopt(sock, SOL_SOCKET, SO_TXTIME, &so_txtime_val, sizeof(so_txtime_val))) {
        perror("setsockopt txtime::");
        exit(1);
    }
}

void server::configure_quiche_server() {
    const struct addrinfo hints = {
        .ai_family = PF_UNSPEC,
        .ai_socktype = SOCK_DGRAM,
        .ai_protocol = IPPROTO_UDP
    };

    struct addrinfo *local;
    if (getaddrinfo(address.c_str(), std::to_string(port).c_str(), &hints, &local) != 0) {
        perror("failed to resolve host");
        exit(1);
    }

    int sock = socket(local->ai_family, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("failed to create socket");
        exit(1);
    }

    if (fcntl(sock, F_SETFL, O_NONBLOCK) != 0) {
        perror("failed to make socket non-blocking");
        exit(1);
    }

    if (bind(sock, local->ai_addr, local->ai_addrlen) < 0) {
        perror("failed to connect socket");
        exit(1);
    }

    setsockopt_txtime(sock);

    auto config = quiche_config_new(QUICHE_PROTOCOL_VERSION);
    if (config == NULL) {
        fprintf(stderr, "failed to create config\n");
        exit(1);
    }

    assert(quiche_config_load_cert_chain_from_pem_file(config, "./cert.crt") == 0);
    assert(quiche_config_load_priv_key_from_pem_file(config, "./cert.key") == 0);
    assert(quiche_config_set_application_protos(config,
        (uint8_t *) "\x0ahq-interop\x05hq-29\x05hq-28\x05hq-27\x08http/0.9", 38) == 0);

    quiche_config_set_initial_congestion_window_packets(config, 10);
    quiche_config_set_max_idle_timeout(config, 5000);
    quiche_config_set_max_recv_udp_payload_size(config, MAX_DATAGRAM_SIZE);
    quiche_config_set_max_send_udp_payload_size(config, MAX_DATAGRAM_SIZE);
    quiche_config_set_initial_max_data(config, 500000000);
    quiche_config_set_initial_max_stream_data_bidi_local(config, 500000000);
    quiche_config_set_initial_max_stream_data_bidi_remote(config, 500000000);
    quiche_config_set_initial_max_stream_data_uni(config, 500000000);
    quiche_config_set_initial_max_streams_bidi(config, 100);
    quiche_config_set_initial_max_streams_uni(config, 100);
    quiche_config_verify_peer(config, false);
    quiche_config_enable_dgram(config, true, 500000, 500000);
    quiche_config_enable_pacing(config, true);

    quiche_config_set_cc_algorithm(config, QUICHE_CC_CUBIC);
    // quiche_config_enable_hystart(config, true);

    struct connections c;
    c.sock = sock;
    c.h = NULL;
    c.local_addr = local->ai_addr;
    c.local_addr_len = local->ai_addrlen;

    conns = &c;

    ev_io watcher;

    struct ev_loop *loop = ev_default_loop(0);

    ev_io_init(&watcher, recv_cb, sock, EV_READ);
    ev_io_start(loop, &watcher);
    watcher.data = &c;

    ev_loop(loop, 0);

    freeaddrinfo(local);
    quiche_config_free(config);
}