#pragma once

#include <chrono>
#include <stdexcept>
#include <thread>

namespace g1_control {
inline constexpr auto kControlPeriod = std::chrono::milliseconds(2);

// Preserve a fixed time grid and skip missed ticks rather than sending a burst.
template<class Clock = std::chrono::steady_clock>
class LoopScheduler {
public:
    using Duration = typename Clock::duration;
    using TimePoint = typename Clock::time_point;
    explicit LoopScheduler(Duration period = kControlPeriod, TimePoint start = Clock::now())
        : period_(period), next_(start) {
        if (period_ <= Duration::zero()) throw std::invalid_argument("Control period must be positive");
    }
    TimePoint Advance(TimePoint now) {
        next_ += period_;
        if (next_ <= now) next_ += period_ * ((now - next_) / period_ + 1);
        return next_;
    }
    void Sleep() { std::this_thread::sleep_until(Advance(Clock::now())); }
private:
    Duration period_;
    TimePoint next_;
};
}  // namespace g1_control
