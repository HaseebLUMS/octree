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
    void server::recv_cb(EV_P_ ev_io *w, int revents);
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

    this->setsockopt_txtime(sock);

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

void server::recv_cb(EV_P_ ev_io *w, int revents) {
    struct conn_io *tmp, *conn_io = NULL;

    uint8_t buf[65535];
    uint8_t out[MAX_DATAGRAM_SIZE];

    while (1) {
        struct sockaddr_storage peer_addr;
        socklen_t peer_addr_len = sizeof(peer_addr);
        memset(&peer_addr, 0, peer_addr_len);

        ssize_t read = recvfrom(conns->sock, buf, sizeof(buf), 0,
                                (struct sockaddr *) &peer_addr,
                                &peer_addr_len);

        if (read < 0) {
            if ((errno == EWOULDBLOCK) || (errno == EAGAIN)) {
                // fprintf(stderr, "recv would block\n");
                break;
            }

            perror("failed to read");
            return;
        }

        uint8_t type;
        uint32_t version;

        uint8_t scid[QUICHE_MAX_CONN_ID_LEN];
        size_t scid_len = sizeof(scid);

        uint8_t dcid[QUICHE_MAX_CONN_ID_LEN];
        size_t dcid_len = sizeof(dcid);

        uint8_t odcid[QUICHE_MAX_CONN_ID_LEN];
        size_t odcid_len = sizeof(odcid);

        uint8_t token[MAX_TOKEN_LEN];
        size_t token_len = sizeof(token);

        int rc = quiche_header_info(buf, read, LOCAL_CONN_ID_LEN, &version,
                                    &type, scid, &scid_len, dcid, &dcid_len,
                                    token, &token_len);
        if (rc < 0) {
            fprintf(stderr, "failed to parse header: %d\n", rc);
            continue;
        }

        HASH_FIND(hh, conns->h, dcid, dcid_len, conn_io);

        if (conn_io == NULL) {
            if (!quiche_version_is_supported(version)) {
                fprintf(stderr, "version negotiation\n");

                ssize_t written = quiche_negotiate_version(scid, scid_len,
                                                           dcid, dcid_len,
                                                           out, sizeof(out));

                if (written < 0) {
                    fprintf(stderr, "failed to create vneg packet: %zd\n",
                            written);
                    continue;
                }

                ssize_t sent = sendto(conns->sock, out, written, 0,
                                      (struct sockaddr *) &peer_addr,
                                      peer_addr_len);
                if (sent != written) {
                    perror("failed to send");
                    continue;
                }

                fprintf(stderr, "sent %zd bytes\n", sent);
                continue;
            }

            if (token_len == 0) {
                fprintf(stderr, "stateless retry\n");

                mint_token(dcid, dcid_len, &peer_addr, peer_addr_len,
                           token, &token_len);

                uint8_t new_cid[LOCAL_CONN_ID_LEN];

                if (gen_cid(new_cid, LOCAL_CONN_ID_LEN) == NULL) {
                    continue;
                }

                ssize_t written = quiche_retry(scid, scid_len,
                                               dcid, dcid_len,
                                               new_cid, LOCAL_CONN_ID_LEN,
                                               token, token_len,
                                               version, out, sizeof(out));

                if (written < 0) {
                    fprintf(stderr, "failed to create retry packet: %zd\n",
                            written);
                    continue;
                }

                ssize_t sent = sendto(conns->sock, out, written, 0,
                                      (struct sockaddr *) &peer_addr,
                                      peer_addr_len);
                if (sent != written) {
                    perror("failed to send");
                    continue;
                }

                fprintf(stderr, "sent %zd bytes\n", sent);
                continue;
            }


            if (!validate_token(token, token_len, &peer_addr, peer_addr_len,
                               odcid, &odcid_len)) {
                fprintf(stderr, "invalid address validation token\n");
                continue;
            }

            conn_io = create_conn(dcid, dcid_len, odcid, odcid_len,
                                  conns->local_addr, conns->local_addr_len,
                                  &peer_addr, peer_addr_len);

            if (conn_io == NULL) {
                continue;
            }
        }

        quiche_recv_info recv_info = {
            (struct sockaddr *)&peer_addr,
            peer_addr_len,

            conns->local_addr,
            conns->local_addr_len,
        };

        ssize_t done = quiche_conn_recv(conn_io->conn, buf, read, &recv_info);

        if (done < 0) {
            fprintf(stderr, "failed to process packet: %zd\n", done);
            continue;
        }

        // fprintf(stderr, "recv %zd bytes\n", done);

        if (quiche_conn_is_established(conn_io->conn)) {
            uint64_t ws = 0;
            quiche_stream_iter *writable = quiche_conn_writable(conn_io->conn);
            while (quiche_stream_iter_next(writable, &ws)) {
                if (processed < reliable_resp.size()) {
                    auto bytes_sent = quiche_conn_stream_send(conn_io->conn, ws, (uint8_t *) reliable_resp.data()+processed, reliable_resp.size()-processed, true);
                    processed += bytes_sent;
                }
            }
            quiche_stream_iter_free(writable);

            uint64_t s = 0;

            quiche_stream_iter *readable = quiche_conn_readable(conn_io->conn);

            while (quiche_stream_iter_next(readable, &s)) {
                fprintf(stderr, "stream %" PRIu64 " is readable\n", s);

                bool fin = false;
                ssize_t recv_len = quiche_conn_stream_recv(conn_io->conn, s,
                                                           buf, sizeof(buf),
                                                           &fin);
                if (recv_len < 0) {
                    break;
                }

                auto msg = "init";

                if (strncmp((char *)buf, "udp", 3) == 0) {
                    msg = "udp";
                } else if (strncmp((char *)buf, "tcp", 3) == 0) {
                    msg = "tcp";
                }

                std::cout << msg << std::endl;

                if (fin) {
                    reliable_resp = "byez\n";
                    std::string unreliable_resp = "";
                    if (strncmp((char *)msg, "tcp", 3) == 0) {
                        std::cout << "Sending Only Reliable Data" << std::endl;
                        reliable_resp = (std::string)tcp_data_buffer.data() + (std::string)udp_data_buffer.data();
                        unreliable_resp = "";
                    } else if (strncmp((char *)msg, "udp", 3) == 0) {
                        std::cout << "Sending Both Reliable (" << tcp_data_buffer.size() << ") & Unreliable Data (" << udp_data_buffer.size() << ")" << std::endl;
                        reliable_resp = (std::string)tcp_data_buffer.data();
                        unreliable_resp = (std::string)udp_data_buffer.data();
                    }

                    auto bytes_sent = quiche_conn_stream_send(conn_io->conn, s, (uint8_t *) reliable_resp.data(), reliable_resp.size(), true);
                    processed = bytes_sent;

                    if (unreliable_resp.size() > 1) {
                        make_chunks_and_send_as_dgrams(conn_io->conn, (uint8_t *) unreliable_resp.data(), unreliable_resp.size());
                    }
                }
            }

            quiche_stream_iter_free(readable);
        }
    }

    HASH_ITER(hh, conns->h, conn_io, tmp) {
        flush_egress(loop, conn_io);

        if (quiche_conn_is_closed(conn_io->conn)) {
            quiche_stats stats;
            quiche_path_stats path_stats;

            quiche_conn_stats(conn_io->conn, &stats);
            quiche_conn_path_stats(conn_io->conn, 0, &path_stats);

            fprintf(stderr, "connection closed, recv=%zu sent=%zu lost=%zu rtt=%" PRIu64 "ns cwnd=%zu retransmitted=%zu\n",
                    stats.recv, stats.sent, stats.lost, path_stats.rtt, path_stats.cwnd, stats.retrans);

            HASH_DELETE(hh, conns->h, conn_io);

            ev_timer_stop(loop, &conn_io->timer);
            quiche_conn_free(conn_io->conn);
            free(conn_io);
        }
    }
}
