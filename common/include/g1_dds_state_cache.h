#pragma once

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace g1_control {
template<class Message>
class StateCache {
public:
    using Clock = std::chrono::steady_clock;
    struct Snapshot {
        Message value{};
        Clock::time_point received_at{};
        bool received = false, valid = false, fault = false;
        std::string error;
        double AgeMs(Clock::time_point now) const {
            return received ? std::chrono::duration<double, std::milli>(now - received_at).count() : -1;
        }
    };
    template<class Validate, class LatchFault>
    void UpdateWithPolicy(const Message& message, Validate validate, LatchFault latch_fault,
                          Clock::time_point now = Clock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (snapshot_.fault) return;
        snapshot_.valid = false;
        try {
            validate(message);
        } catch (const std::exception& error) {
            snapshot_.error = error.what();
            snapshot_.fault = latch_fault();
            return;
        }
        snapshot_.value = message;
        snapshot_.received_at = now;
        snapshot_.received = snapshot_.valid = true;
        snapshot_.error.clear();
    }
    template<class Validate>
    void Update(const Message& message, Validate validate, bool latch_fault = false,
                Clock::time_point now = Clock::now()) {
        UpdateWithPolicy(message, validate, [latch_fault] { return latch_fault; }, now);
    }
    void Update(const Message& message, Clock::time_point now = Clock::now()) {
        Update(message, [](const Message&) {}, false, now);
    }
    Snapshot Read() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }
    template<class Duration>
    bool ReadFresh(Message& message, Duration max_age, Clock::time_point now = Clock::now()) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!snapshot_.valid || now - snapshot_.received_at > max_age) return false;
        message = snapshot_.value;
        return true;
    }
    std::string Error() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_.error;
    }
private:
    mutable std::mutex mutex_;
    Snapshot snapshot_;
};

// Close the subscriber before destroying the data referenced by its callback.
template<class Message>
class DdsStateCache {
public:
    explicit DdsStateCache(const std::string& topic) : subscriber_(topic) {}
    DdsStateCache(const DdsStateCache&) = delete;
    DdsStateCache& operator=(const DdsStateCache&) = delete;
    ~DdsStateCache() { Close(); }
    template<class Validate, class LatchFault>
    void Start(Validate validate, LatchFault latch_fault) {
        subscriber_.InitChannel([this, validate, latch_fault](const void* message) {
            cache_.UpdateWithPolicy(*static_cast<const Message*>(message), validate, latch_fault);
        }, 1);
        started_ = true;
    }
    void Start() { Start([](const Message&) {}, [] { return false; }); }
    void Close() {
        if (started_) { subscriber_.CloseChannel(); started_ = false; }
    }
    auto Read() const { return cache_.Read(); }
    std::string Error() const { return cache_.Error(); }
    template<class Duration> bool ReadFresh(Message& message, Duration max_age) const {
        return cache_.ReadFresh(message, max_age);
    }
private:
    StateCache<Message> cache_;
    unitree::robot::ChannelSubscriber<Message> subscriber_;
    bool started_ = false;
};
}  // namespace g1_control
