#pragma once
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace g1_parkour {
// Single owner of policy history and inference. Mailboxes only hold snapshots;
// the mutex is never held while resetting, preprocessing or running the model.
template<class Observation, class Target>
class PolicyWorker {
public:
    using Clock = std::chrono::steady_clock;
    using Velocity = std::array<float,3>;
    struct Result {
        Target target;
        Clock::time_point input_at, completed_at;
    };
    static constexpr auto period = std::chrono::milliseconds(20);
    static constexpr auto target_timeout = std::chrono::milliseconds(60);
private:
    std::function<Target(const Observation&,const Velocity&)> step_;
    std::function<void()> reset_;
    const bool synchronized_source_;
    const Clock::duration period_, target_timeout_;
    static Clock::duration ValidPeriod(Clock::duration value) {
        if(value<std::chrono::milliseconds(2)) throw std::runtime_error("Invalid policy worker period");
        return value;
    }
    std::mutex mutex_;
    std::condition_variable changed_;
    bool closed_=false, active_=false, have_input_=false, strict_=false;
    uint64_t epoch_=0;
    Observation observation_{};
    Velocity velocity_{};
    Clock::time_point submitted_at_{}, started_at_{};
    std::optional<Result> result_;
    std::exception_ptr error_;
    std::thread thread_;

    void Run() {
        std::unique_lock<std::mutex> lock(mutex_);
        uint64_t generation=0, last_sequence=0;
        bool have_sequence=false;
        Clock::time_point next{};
        while(!closed_) {
            changed_.wait(lock,[&] {return closed_ || (active_ && have_input_ && !error_);});
            if(closed_) break;
            const auto epoch=epoch_;
            if(generation!=epoch) {
                generation=epoch;have_sequence=false;
                lock.unlock();
                try {reset_();}
                catch(...) {
                    lock.lock();
                    if(epoch_==epoch && active_) error_=std::current_exception();
                    continue;
                }
                lock.lock();
                if(closed_ || !active_ || epoch_!=epoch) continue;
                next=Clock::now();
            }
            if(Clock::now()<next) {
                changed_.wait_until(lock,next,[&] {return closed_ || !active_ || epoch_!=epoch;});
                continue;
            }
            const auto observation=observation_;
            const auto velocity=velocity_;
            const auto input_at=submitted_at_;
            const bool strict=strict_;
            // Simulator history advances only for a new synchronized observation.
            if(synchronized_source_ && have_sequence && observation.sequence==last_sequence) {
                next+=period_;
                const auto after=Clock::now();
                if(next<=after) next+=period_*((after-next)/period_+1);
                continue;
            }
            lock.unlock();
            try {
                const auto before=Clock::now();
                if(before-input_at>=target_timeout_)
                    throw std::runtime_error("Policy input snapshot stale");
                auto target=step_(observation,velocity);
                const auto completed=Clock::now();
                if(strict && completed-before>period_)
                    throw std::runtime_error("Policy preprocessing/inference exceeded "+
                        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(period_).count())+"ms period");
                lock.lock();
                if(epoch_==epoch && active_ && !closed_) {
                    result_=Result{target,input_at,completed};
                    last_sequence=observation.sequence;have_sequence=true;
                }
            } catch(...) {
                lock.lock();
                if(epoch_==epoch && active_ && !closed_) error_=std::current_exception();
            }
            // Fixed deadlines; skip missed periods instead of bursting to catch up.
            next+=period_;
            const auto after=Clock::now();
            if(next<=after) next+=period_*((after-next)/period_+1);
        }
    }
public:
    PolicyWorker(std::function<Target(const Observation&,const Velocity&)> step,
                 std::function<void()> reset, bool synchronized_source=false, Clock::duration update_period=period)
        :step_(std::move(step)),reset_(std::move(reset)),synchronized_source_(synchronized_source),
         period_(ValidPeriod(update_period)),target_timeout_(3*period_),thread_([this] {Run();}) {}
    ~PolicyWorker() {
        {std::lock_guard<std::mutex> lock(mutex_);closed_=true;}
        changed_.notify_all();
        if(thread_.joinable()) thread_.join();
    }
    PolicyWorker(const PolicyWorker&)=delete;
    PolicyWorker& operator=(const PolicyWorker&)=delete;
    void Begin(bool strict) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++epoch_;active_=true;have_input_=false;strict_=strict;
        result_.reset();error_=nullptr;started_at_=Clock::now();
        changed_.notify_all();
    }
    void Pause() {
        std::lock_guard<std::mutex> lock(mutex_);
        ++epoch_;active_=false;have_input_=false;result_.reset();error_=nullptr;
        changed_.notify_all();
    }
    void Submit(const Observation& observation,const Velocity& velocity) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!active_) return;
        observation_=observation;velocity_=velocity;submitted_at_=Clock::now();have_input_=true;
        changed_.notify_all();
    }
    std::optional<Result> Latest() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(error_) std::rethrow_exception(error_);
        if(!active_) return std::nullopt;
        const auto now=Clock::now();
        if(result_ ? now-result_->completed_at>=target_timeout_ || now-result_->input_at>=target_timeout_
                   : now-started_at_>=target_timeout_)
            throw std::runtime_error("Policy target stale (three-period watchdog)");
        return result_;
    }
};
} // namespace g1_parkour
