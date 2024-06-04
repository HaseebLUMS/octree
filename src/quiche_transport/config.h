#include <iostream>
#include <chrono>
#include <fstream>

#define MAX_DATAGRAM_SIZE 1350
#define MAX_PKT_SIZE 1310 // Used for sending unreliable datagrams


const int RELIABLE_DATA_SIZE   = 100 * 1024 * 1024;
const int UNRELIABLE_DATA_SIZE = 100 * 1024 * 1024;
const int FRAME_SIZE           = 1 * 1024 * 1024;  // Bytes
const char* LOGS_LOCATION      = "./../quic_benchmarks/data";  // assuming ./ is the build directory

void redirectClogToDevNull() {
    static std::ofstream devNull("/dev/null");
    std::clog.rdbuf(devNull.rdbuf());
}

// In microseconds
int64_t get_current_time() {
    auto currentTime = std::chrono::system_clock::now();
    auto durationSinceEpoch = currentTime.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(durationSinceEpoch);
    int64_t microsecondsCount = microseconds.count();
    return microsecondsCount;
}
