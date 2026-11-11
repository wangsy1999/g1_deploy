#pragma once

#include <cerrno>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace g1_control {
// Read single keys without Enter; retain terminal signal handling and restore on exit.
class Keyboard {
public:
    explicit Keyboard(int fd = STDIN_FILENO) : fd_(fd) {
        if (!isatty(fd_)) return;
        if (tcgetattr(fd_, &original_) != 0)
            throw std::runtime_error("Cannot read terminal settings");
        auto settings = original_;
        settings.c_lflag &= ~(ICANON | ECHO);
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &settings) != 0)
            throw std::runtime_error("Cannot enable keyboard input");
        restore_ = true;
    }
    Keyboard(const Keyboard&) = delete;
    Keyboard& operator=(const Keyboard&) = delete;
    ~Keyboard() {
        if (restore_) tcsetattr(fd_, TCSANOW, &original_);
    }
    char Read(bool eof_is_exit = true) {
        pollfd input{fd_, POLLIN, 0};
        const int ready = poll(&input, 1, 0);
        if (ready < 0) {
            if (errno == EINTR) return 0;
            throw std::runtime_error("Cannot poll keyboard input");
        }
        if (!ready) return 0;
        if (input.revents & (POLLERR | POLLNVAL))
            throw std::runtime_error("Keyboard input closed");
        char key = 0;
        const auto count = read(fd_, &key, 1);
        if (count == 0) return eof_is_exit ? ' ' : 0;
        if (count < 0 && errno != EINTR && errno != EAGAIN)
            throw std::runtime_error("Cannot read keyboard input");
        return count > 0 ? key : 0;
    }
private:
    int fd_;
    termios original_{};
    bool restore_ = false;
};
}  // namespace g1_control
