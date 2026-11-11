#include "g1_loop_scheduler.h"
#include "g1_dds_state_cache.h"
#include "g1_monitor_publisher.h"
#include "g1_joint_mapping.h"
#include <atomic>
#include <iostream>
#include <thread>
#include <unitree/idl/hg/LowState_.hpp>

namespace {
using Clock = std::chrono::steady_clock;
using State = unitree_hg::msg::dds_::LowState_;
using Command = unitree_hg::msg::dds_::LowCmd_;
void Require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
template<class F> void Reject(F function) {
    bool rejected = false;
    try { function(); } catch (const std::exception&) { rejected = true; }
    Require(rejected, "Invalid input was accepted");
}
void CheckScheduling() {
    const Clock::time_point start{};
    const auto ms = std::chrono::milliseconds(1);
    g1_control::LoopScheduler<> loop(2 * ms, start);
    Require(loop.Advance(start + ms) == start + 2 * ms, "Schedule drifted");
    Require(loop.Advance(start + 4 * ms) == start + 6 * ms, "Missed boundary was not skipped");
    Require(loop.Advance(start + 13 * ms) == start + 14 * ms, "Stall caused catch-up ticks");
    Require(loop.Advance(start + 1000 * ms) == start + 1002 * ms, "Long stall changed time grid");
    Reject([] { g1_control::LoopScheduler<> loop(Clock::duration::zero()); });
}
void CheckCache() {
    using Pair = std::array<int, 2>;
    g1_control::StateCache<Pair> cache;
    const Clock::time_point start{};
    const auto timeout = std::chrono::milliseconds(10);
    Pair value{};
    Require(!cache.ReadFresh(value, timeout, start), "Empty cache was fresh");
    cache.Update(Pair{1, 1}, start);
    Require(cache.ReadFresh(value, timeout, start + timeout) && value == Pair{1, 1}, "Freshness boundary changed");
    Require(!cache.ReadFresh(value, timeout, start + timeout + std::chrono::nanoseconds(1)), "Stale state was accepted");
    auto invalid = [](const Pair&) { throw std::runtime_error("bad state"); };
    cache.Update(Pair{2, 2}, invalid);
    Require(!cache.Read().valid && !cache.Read().fault && cache.Error() == "bad state", "Preview validation latched a fault");
    cache.Update(Pair{3, 3});
    Require(cache.Read().valid && cache.Error().empty(), "Cache did not recover");
    bool armed = false;
    cache.UpdateWithPolicy(Pair{4, 4}, [&](const Pair&) { armed = true; throw std::runtime_error("armed fault"); }, [&] { return armed; });
    cache.Update(Pair{5, 5});
    Require(cache.Read().fault && !cache.Read().valid && cache.Read().value == Pair{3, 3}, "Latched fault recovered without restart");
    g1_control::StateCache<Pair> concurrent;
    std::atomic<bool> done{false};
    std::thread writer([&] { for (int n = 0; n < 5000; ++n) concurrent.Update(Pair{n, n}); done = true; });
    bool consistent = true;
    while (!done) { const auto sample = concurrent.Read(); consistent &= sample.value[0] == sample.value[1]; }
    writer.join();
    Require(consistent, "Snapshot contained mixed messages");
}
void CheckMapping() {
    const std::array<int, 3> ids{2, 0, 1};
    g1_joints::ValidatePermutation(ids);
    Require(g1_joints::ToMotorOrder(std::array<float, 3>{10, 20, 30}, ids) == std::array<float, 3>{20, 30, 10}, "Wrong motor order");
    Require(g1_joints::ToMotorOrder(std::array<float, 3>{10, 20, 30}, ids, std::array<float, 3>{-1, 1, -1}) ==
            std::array<float, 3>{20, -30, -10}, "Wrong signed motor order");
    Reject([] { g1_joints::ValidatePermutation(std::array<int, 3>{0, 0, 2}); });
    Reject([] { g1_joints::ValidatePermutation(std::array<int, 3>{0, 1, 3}); });
    Reject([] { g1_joints::ValidatePermutation(std::array<int, 3>{0, 1, -1}); });
    std::vector<std::string> names(g1_joints::kDdsNames.rbegin(), g1_joints::kDdsNames.rend());
    const auto mapped = g1_joints::FromNames(names);
    for (int i = 0; i < 29; ++i) Require(mapped[i] == 28 - i, "Joint name mapping changed");
    names[0] = names[1]; Reject([&] { g1_joints::FromNames(names); });
    names[0] = "unknown"; Reject([&] { g1_joints::FromNames(names); });
}
void CheckMessages() {
    Command command{};
    command.mode_machine() = 5; command.motor_cmd()[0].q() = .7f;
    command.motor_cmd()[0].mode() = 1; command.motor_cmd()[34].mode() = 1;
    const auto preview = g1_control::PreviewTarget(command);
    for (const auto& motor : preview.motor_cmd()) Require(motor.mode() == 0, "Preview retained actuator mode");
    Require(preview.motor_cmd()[0].q() == .7f && preview.mode_machine() == 5 && preview.crc() == g1_hardware::Crc(preview), "Preview changed targets/metadata or CRC");
    Require(command.motor_cmd()[0].mode() == 1 && command.motor_cmd()[34].mode() == 1, "Preview modified original command");
    for (bool enabled : {false, true}) {
        const auto status = g1_control::EnableStatus(enabled);
        Require(status.crc() == g1_hardware::Crc(status), "Enable CRC invalid");
        for (int i = 0; i < 29; ++i) Require(status.motor_cmd()[i].mode() == (enabled ? 1 : 0), "Enable mode invalid");
    }
}
template<class Send, class Ready> void WaitFor(Send send, Ready ready) {
    for (int i = 0; i < 500; ++i) {
        send();
        if (ready()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("DDS test message not received");
}
void CheckDds() {
    unitree::robot::ChannelFactory::Instance()->Init(198, "lo");
    g1_control::DdsStateCache<State> receiver("rt/g1/test/state_cache");
    receiver.Start([](const State& state) { if (state.mode_machine() != 5) throw std::runtime_error("wrong mode"); }, [] { return false; });
    unitree::robot::ChannelPublisher<State> writer("rt/g1/test/state_cache"); writer.InitChannel();
    State state{};
    WaitFor([&] { writer.Write(state); }, [&] { return !receiver.Error().empty(); });
    state.mode_machine() = 5; state.motor_state()[0].q() = .25f;
    WaitFor([&] { writer.Write(state); }, [&] { return receiver.Read().valid; });
    Require(receiver.Read().value.motor_state()[0].q() == .25f, "DDS cache lost state");
    receiver.Close();
    const auto closed_at = receiver.Read().received_at;
    for (int i = 0; i < 10; ++i) { writer.Write(state); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    Require(receiver.Read().received_at == closed_at, "Callback ran after close");

    g1_control::StateCache<Command> targets, previews, enables;
    unitree::robot::ChannelSubscriber<Command> target_sub("rt/g1/reset_target"), preview_sub("rt/g1/dry_run_target"), enable_sub("rt/g1/reset_enable");
    target_sub.InitChannel([&](const void* msg) { targets.Update(*static_cast<const Command*>(msg)); }, 1);
    preview_sub.InitChannel([&](const void* msg) { previews.Update(*static_cast<const Command*>(msg)); }, 1);
    enable_sub.InitChannel([&](const void* msg) { enables.Update(*static_cast<const Command*>(msg)); }, 1);
    g1_control::MonitorPublisher telemetry; telemetry.Start();
    Command command{}; command.motor_cmd()[0].q() = .7f; command.motor_cmd()[0].mode() = 1;
    WaitFor([&] { telemetry.PublishTarget(command); telemetry.PublishPreview(command); telemetry.PublishEnable(true); },
            [&] { return targets.Read().valid && previews.Read().valid && enables.Read().valid; });
    const auto target = targets.Read().value, preview = previews.Read().value, enabled = enables.Read().value;
    Require(target.motor_cmd()[0].q() == .7f && target.motor_cmd()[0].mode() == 1 && target.crc() == g1_hardware::Crc(target), "Published target invalid");
    Require(preview.motor_cmd()[0].q() == .7f && preview.motor_cmd()[0].mode() == 0 && preview.crc() == g1_hardware::Crc(preview), "Published preview invalid");
    Require(enabled.motor_cmd()[0].mode() == 1 && enabled.crc() == g1_hardware::Crc(enabled), "Published enable invalid");
}
}  // namespace
int main() {
    try {
        CheckScheduling(); CheckCache(); CheckMapping(); CheckMessages(); CheckDds();
        std::cout << "Shared scheduling, state cache, telemetry and joint mapping checks passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
