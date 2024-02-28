class server {
public:
    server() {}
    int send_frame();
    void listen_for_connections();
    void serve_client();
};

int server::send_frame() {
    // serialize the reliable and unreliable parts
    // create necessary metadata (size of unreliable parts ?)
    // enqueue the data in all the client queues
}

void server::listen_for_connections() {
    // accept new connections
    // for each new connections, spin a thread `serve_client`, make a new client queue
}

void server::serve_client() {
    // keep watching the respective queue for new data
    // upon new data, send it to the client using quiche
}