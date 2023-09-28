const char* SERVER_IP = "127.0.0.1";
const char* CLIENT_IP = "127.0.0.1";

const int PACKET_RATE_Mbps = 200;

const int SERVER_TCP_PORT = 9999;
const int SERVER_UDP_PORT = 8888;

const int CLIENT_TCP_PORT = 9998;
const int CLIENT_UDP_PORT = 8887;

const int UDP_SOCKET_WAIT = 20000; // microseconds (us)

const int BUFFER_SIZE = 1470;
const int BUFFER_SIZE_WITH_EXTRA_ROOM = BUFFER_SIZE + 100;

const int RELIABLE_DATA_SIZE = 1 * 1024 * 1024;
const int UNRELIABLE_DATA_SIZE = 2 * 1024 * 1024;

const int CUSHION_IN_SLEEP_CALCULATIONS = 10; // microseconds