#pragma once

#include <chrono>

namespace g1_control {
inline constexpr auto kStartupTimeout = std::chrono::seconds(10);

// A stop from either source always wins; other remote actions take precedence.
inline char SelectAction(char keyboard, char remote) {
    if (keyboard == ' ' || remote == ' ') return ' ';
    return remote ? remote : keyboard;
}
}  // namespace g1_control
