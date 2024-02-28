class client {
public:
    client() {}
    int attempt_connection();
};

int client::attempt_connection() {
    // attempts to connect to the server
    // upon connection, spins `receive_data` and `process_data` threads
}

void receive_data() {
    // will keep on receiving data from the server
    // any received data will be passed to `process_data` via a queue
}

void process_data() {
    // will keep on checking for new data in the queue
    // upon any new data, it will try to construct a frame
    // when a frame is completed, it will be pushed to `available_frames` queue
}

void get_frame() {
    // will check the `available_frames` queue and return the first stored frame
}