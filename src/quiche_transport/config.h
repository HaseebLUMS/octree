const int RELIABLE_DATA_SIZE = 99 * 1024 * 1024;
const int UNRELIABLE_DATA_SIZE = 1 * 1024 * 1024;

void redirectClogToDevNull() {
    static std::ofstream devNull("/dev/null");
    std::clog.rdbuf(devNull.rdbuf());
}

int64_t get_current_time() {
    auto currentTime = std::chrono::system_clock::now();
    auto durationSinceEpoch = currentTime.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(durationSinceEpoch);
    int64_t microsecondsCount = microseconds.count();
    return microsecondsCount;
}
