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

namespace g1_inference {
// 控制线程提交最新快照，工作线程独占策略历史并执行推理。
// 邮箱只保留最新输入和结果；重置、预处理和模型推理期间不持有邮箱锁。
// Observation 需要提供 sequence 字段，供同步仿真模式识别重复帧。
template<class Observation, class Target>
class PolicyWorker {
public:
    using Clock = std::chrono::steady_clock;
    using Velocity = std::array<float, 3>; // 前进速度、侧向速度、转向角速度
    struct Result {
        Target target;
        Clock::time_point input_at;     // 输入快照进入邮箱的时刻
        Clock::time_point completed_at; // 本次推理完成的时刻
    };
    static constexpr auto period = std::chrono::milliseconds(20);
    static constexpr auto target_timeout = std::chrono::milliseconds(60);

private:
    std::function<Target(const Observation&, const Velocity&)> step_;
    std::function<void()> reset_;
    const bool synchronized_source_;
    const Clock::duration period_;
    const Clock::duration target_timeout_;

    static Clock::duration ValidPeriod(Clock::duration value) {
        if (value < std::chrono::milliseconds(2)) {
            throw std::runtime_error("Invalid policy worker period");
        }
        return value;
    }

    // 以下邮箱状态由 mutex_ 保护。Begin/Pause 都递增激活编号，
    // 使切换之前尚未完成的推理结果和异常失效。
    std::mutex mutex_;
    std::condition_variable changed_;
    bool closed_ = false;
    bool active_ = false;
    bool have_input_ = false;
    bool enforce_deadline_ = false;
    uint64_t activation_id_ = 0;
    Observation observation_{};
    Velocity velocity_{};
    Clock::time_point submitted_at_{};
    Clock::time_point started_at_{};
    std::optional<Result> result_;
    std::exception_ptr error_;
    std::thread thread_;

    void Run() {
        std::unique_lock<std::mutex> lock(mutex_);
        uint64_t reset_activation_id = 0;
        uint64_t last_sequence = 0;
        bool have_sequence = false;
        Clock::time_point next_deadline{};
        while (!closed_) {
            changed_.wait(lock, [&] {
                return closed_ || (active_ && have_input_ && !error_);
            });
            if (closed_) break;
            const auto activation_id = activation_id_;

            // 每次激活后的第一份输入到达时，先在推理线程重置策略历史。
            if (reset_activation_id != activation_id) {
                reset_activation_id = activation_id;
                have_sequence = false;
                lock.unlock();
                try {
                    reset_();
                } catch (...) {
                    lock.lock();
                    if (activation_id_ == activation_id && active_) {
                        error_ = std::current_exception();
                    }
                    continue;
                }
                lock.lock();
                if (closed_ || !active_ || activation_id_ != activation_id) continue;
                next_deadline = Clock::now();
            }
            if (Clock::now() < next_deadline) {
                changed_.wait_until(lock, next_deadline, [&] {
                    return closed_ || !active_ || activation_id_ != activation_id;
                });
                continue;
            }

            // 在锁内复制输入，随后释放锁，避免模型阻塞控制线程提交新状态。
            const auto observation = observation_;
            const auto velocity = velocity_;
            const auto input_at = submitted_at_;
            const bool enforce_deadline = enforce_deadline_;
            // 同步仿真中，同一帧只推理一次，避免重复推进策略历史。
            if (synchronized_source_ && have_sequence && observation.sequence == last_sequence) {
                next_deadline += period_;
                const auto now = Clock::now();
                if (next_deadline <= now) {
                    const auto missed_periods = (now - next_deadline) / period_ + 1;
                    next_deadline += period_ * missed_periods;
                }
                continue;
            }
            lock.unlock();
            try {
                const auto inference_started_at = Clock::now();
                if (inference_started_at - input_at >= target_timeout_) {
                    throw std::runtime_error("Policy input snapshot stale");
                }
                auto target = step_(observation, velocity);
                const auto completed_at = Clock::now();
                if (enforce_deadline && completed_at - inference_started_at > period_) {
                    throw std::runtime_error("Policy preprocessing/inference exceeded " +
                        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(period_).count()) +
                        "ms period");
                }
                lock.lock();
                // 推理期间可能发生暂停或重新激活，只发布当前激活的结果。
                if (activation_id_ == activation_id && active_ && !closed_) {
                    result_ = Result{target, input_at, completed_at};
                    last_sequence = observation.sequence;
                    have_sequence = true;
                }
            } catch (...) {
                lock.lock();
                if (activation_id_ == activation_id && active_ && !closed_) {
                    error_ = std::current_exception();
                }
            }
            // 按固定时间轴调度；超时后跳过错过的周期，避免连续补跑。
            next_deadline += period_;
            const auto now = Clock::now();
            if (next_deadline <= now) {
                const auto missed_periods = (now - next_deadline) / period_ + 1;
                next_deadline += period_ * missed_periods;
            }
        }
    }

public:
    PolicyWorker(std::function<Target(const Observation&, const Velocity&)> step,
                 std::function<void()> reset, bool synchronized_source = false,
                 Clock::duration update_period = period)
        : step_(std::move(step)), reset_(std::move(reset)), synchronized_source_(synchronized_source),
          period_(ValidPeriod(update_period)), target_timeout_(3 * period_), thread_([this] { Run(); }) {}

    ~PolicyWorker() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        changed_.notify_all();
        if (thread_.joinable()) thread_.join();
    }
    PolicyWorker(const PolicyWorker&) = delete;
    PolicyWorker& operator=(const PolicyWorker&) = delete;

    // 清空邮箱并开启一次新的激活；strict 要求单次预处理和推理不超过周期。
    void Begin(bool strict) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++activation_id_;
        active_ = true;
        have_input_ = false;
        enforce_deadline_ = strict;
        result_.reset();
        error_ = nullptr;
        started_at_ = Clock::now();
        changed_.notify_all();
    }

    // 立即使旧结果失效；正在运行的推理完成后会自行丢弃其结果。
    void Pause() {
        std::lock_guard<std::mutex> lock(mutex_);
        ++activation_id_;
        active_ = false;
        have_input_ = false;
        result_.reset();
        error_ = nullptr;
        changed_.notify_all();
    }

    // 覆盖最新快照，不排队积压；暂停时忽略提交。
    void Submit(const Observation& observation, const Velocity& velocity) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) return;
        observation_ = observation;
        velocity_ = velocity;
        submitted_at_ = Clock::now();
        have_input_ = true;
        changed_.notify_all();
    }

    // 将推理异常交给控制线程处理。首次结果、结果年龄和输入年龄
    // 都受三个推理周期的看门狗约束，防止旧快照被反复推理后续期。
    std::optional<Result> Latest() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (error_) std::rethrow_exception(error_);
        if (!active_) return std::nullopt;
        const auto now = Clock::now();
        bool timed_out;
        if (result_) {
            const bool result_stale = now - result_->completed_at >= target_timeout_;
            const bool input_stale = now - result_->input_at >= target_timeout_;
            timed_out = result_stale || input_stale;
        } else {
            timed_out = now - started_at_ >= target_timeout_;
        }
        if (timed_out) {
            throw std::runtime_error("Policy target stale (three-period watchdog)");
        }
        return result_;
    }
};
} // namespace g1_inference
