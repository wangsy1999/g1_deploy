#pragma once

#include <csignal>

namespace g1_control {
// The signal handler only writes sig_atomic_t; control loops perform the cleanup.
inline volatile std::sig_atomic_t stop_requested = 0;
inline void RequestStop(int) { stop_requested = 1; }
inline bool StopRequested() { return stop_requested != 0; }
inline void InstallSignalHandlers() {
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    std::signal(SIGHUP, RequestStop);
}
}  // namespace g1_control
