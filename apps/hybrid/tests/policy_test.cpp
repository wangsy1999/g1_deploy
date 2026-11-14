#include "g1_hybrid_policy.h"
#include <iostream>
#include <limits>

namespace {
using g1_hybrid::Mode;
using Targets = g1_parkour_app::MotorTargets;
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
bool Equal(const Targets& a, const Targets& b) {
    return a.position == b.position && a.kp == b.kp && a.kd == b.kd;
}
void CheckSelector() {
    g1_hybrid::ModeSelector selector({});
    for (int i = 0; i < 9; ++i) Require(selector.Update(.3f, .02) == Mode::Dwaq, "Forward confirmation was skipped");
    Require(selector.Update(.3f, .02) == Mode::Parkour, "Forward never selected Parkour");
    Require(selector.Update(.1f, .02) == Mode::Parkour, "Hysteresis did not retain Parkour");
    Require(selector.Update(0, .02) == Mode::Dwaq, "Standing did not return to DWaQ");
    for (int i = 0; i < 9; ++i) selector.Update(.3f, .02);
    selector.Update(-.1f, .02);
    Require(selector.Update(.3f, .02) == Mode::Dwaq, "Interrupted confirmation was retained");
    selector.Reset();
    for (int i = 0; i < 30; ++i) Require(selector.Update(-.2f, .02) == Mode::Dwaq, "Backward selected Parkour");
    bool failed = false;
    try { selector.Update(std::numeric_limits<float>::quiet_NaN(), .02); } catch (...) { failed = true; }
    Require(failed, "Non-finite velocity accepted");
}
g1_hybrid::Policy::Observation Observation(const g1_hybrid::Policy& hybrid) {
    g1_hybrid::Policy::Observation observation;
    const auto defaults = hybrid.DefaultTargets();
    const auto& camera = hybrid.CameraPolicy();
    observation.lowstate.imu_state().quaternion() = {1, .01f, -.02f, 0};
    observation.lowstate.imu_state().gyroscope() = {.1f, .2f, .3f};
    observation.gravity = {0, 0, -1};
    observation.omega = {.4f, .5f, .6f}; // Deliberately different torso/pelvis inputs.
    observation.depth.fill(2.5f);
    for (int joint = 0; joint < 29; ++joint) {
        observation.lowstate.motor_state()[joint].q() = defaults.position[joint];
        observation.lowstate.motor_state()[joint].dq() = .01f * joint;
        observation.q[joint] = camera.sign[joint] * defaults.position[camera.ids[joint]];
        observation.dq[joint] = camera.sign[joint] * .01f * camera.ids[joint];
    }
    return observation;
}
void CheckDwaqUnchanged() {
    g1_hybrid::Policy hybrid(G1_HYBRID_TEST_CONFIG);
    g1_policy::Policy standalone(G1_DWAQ_TEST_CONFIG);
    auto observation = Observation(hybrid);
    for (int tick = 0; tick < 100; ++tick) {
        const std::array<float, 3> command{tick < 50 ? 0.f : -.2f, 0, .2f};
        auto state = g1_hybrid::Policy::DwaqState(observation.lowstate);
        Require(state.omega[0] == .1f, "DWaQ pelvis IMU was replaced by torso data");
        const auto actual = hybrid.Step(observation, command);
        Require(actual.position == standalone.Step(state, command), "Hybrid DWaQ targets differ from standalone");
        Require(actual.kp == standalone.kp && actual.kd == standalone.kd, "DWaQ gains changed");
        Require(hybrid.SelectedMode() == Mode::Dwaq, "Idle/backward mode changed");
    }
}
void CheckDwaqTrainingHistory() {
    g1_policy::Policy policy(G1_DWAQ_TEST_CONFIG);
    g1_policy::State state;
    state.q = policy.DefaultDds();
    // Distinct full frames catch both term-major packing and reversed time order.
    for (int tick = 1; tick <= 7; ++tick) {
        state.omega = {float(tick), float(tick + 10), float(tick + 20)};
        const auto history = policy.Observe(state, {float(tick), 0, 0});
        for (int frame = 0; frame < 5; ++frame) {
            const float expected = float(std::max(1, tick - 4 + frame));
            Require(history[96 * frame] == expected && history[96 * frame + 1] == expected + 10 &&
                    history[96 * frame + 5] == -1 && history[96 * frame + 6] == expected,
                    "DWaQ history does not match training's oldest-first whole frames");
        }
    }
    policy.Reset();
    const auto reset_history = policy.Observe(state, {7, 0, 0});
    for (int frame = 0; frame < 5; ++frame)
        Require(reset_history[96 * frame] == 7, "DWaQ reset retained previous history");
    Require(policy.kp[13] == 50 && policy.kp[14] == 50, "DWaQ waist gains differ from training");
    g1_policy::Joints action{};
    action.fill(1000);
    const auto targets = policy.ProcessActions(action);
    // 部署配置允许限制最终目标角度，不能固定假设目标始终不限幅。
    const auto cfg = YAML::LoadFile(G1_DWAQ_TEST_CONFIG);
    const auto action_cfg = cfg["actions"]["JointPositionAction"];
    const auto ids = cfg["joint_ids_map"].as<std::vector<int>>();
    for (int joint = 0; joint < 29; ++joint) {
        const float raw_clip = action_cfg["raw_clip"].as<float>(std::numeric_limits<float>::max());
        const float applied = std::clamp(action[joint], -raw_clip, raw_clip);
        float expected = action_cfg["offset"][joint].as<float>() +
                         action_cfg["scale"][joint].as<float>() * applied;
        if (!action_cfg["clip"].IsNull()) {
            expected = std::clamp(expected, action_cfg["clip"][joint][0].as<float>(),
                                 action_cfg["clip"][joint][1].as<float>());
        }
        Require(targets[ids[joint]] == expected, "DWaQ target differs from configured action processing");
    }
}
void CheckSwitchesAndReset() {
    g1_hybrid::Policy hybrid(G1_HYBRID_TEST_CONFIG);
    g1_parkour::Policy parkour(G1_PARKOUR_TEST_CONFIG);
    g1_policy::Policy dwaq(G1_DWAQ_TEST_CONFIG);
    auto observation = Observation(hybrid);
    int frame = 0;
    // Independent references run on every frame, including while their output is not selected.
    for (const auto command : {std::array<float, 3>{0, 0, 0}, {-.2f, 0, 0}, {.3f, 0, 0},
                               {0, 0, 0}, {.3f, 0, 0}, {-.2f, 0, 0}}) {
        for (int tick = 0; tick < 35; ++tick, ++frame) {
            observation.q[0] += .0001f;
            observation.lowstate.motor_state()[0].q() += .0002f;
            observation.depth[0] = 1.f + .001f * frame;
            const auto result = hybrid.Step(observation, command);
            const Targets dwaq_expected{
                dwaq.Step(g1_hybrid::Policy::DwaqState(observation.lowstate), command), dwaq.kp, dwaq.kd};
            auto forward_command = command;
            forward_command[0] = std::max(0.f, forward_command[0]);
            const Targets parkour_expected{
                parkour.Step(observation, forward_command),
                g1_joints::ToMotorOrder(parkour.kp, parkour.ids),
                g1_joints::ToMotorOrder(parkour.kd, parkour.ids)};
            const auto& expected = hybrid.SelectedMode() == Mode::Dwaq ? dwaq_expected : parkour_expected;
            Require(Equal(result, expected), "Selected output differs from continuously running reference; histories were reset or left stale");
        }
        Require(hybrid.SelectedMode() == (command[0] > .15f ? Mode::Parkour : Mode::Dwaq),
                "Continuous inference changed mode selection");
    }
    hybrid.Reset(); dwaq.Reset(); parkour.Reset();
    Require(hybrid.SelectedMode() == Mode::Dwaq, "Reset retained forward mode");
    Require(hybrid.Step(observation, {0, 0, 0}).position ==
            dwaq.Step(g1_hybrid::Policy::DwaqState(observation.lowstate), {0, 0, 0}), "Reset retained stale DWaQ history");
    parkour.Step(observation, {0, 0, 0});
    for (int tick = 0; tick < 10; ++tick) {
        hybrid.Step(observation, {.3f, 0, 0});
        parkour.Step(observation, {.3f, 0, 0});
        if (tick == 9) Require(hybrid.SelectedMode() == Mode::Parkour, "Forward entry failed after reset");
    }
    const auto actual = hybrid.Step(observation, {.3f, 0, 0});
    Require(actual.position == parkour.Step(observation, {.3f, 0, 0}), "Reset retained stale Parkour history");
}

void CheckWarmupDoesNotLeakHistory() {
    g1_hybrid::Policy policy(G1_HYBRID_TEST_CONFIG);
    g1_hybrid::Policy reference(G1_HYBRID_TEST_CONFIG);
    const auto observation = Observation(policy);
    for (int tick = 0; tick < 20; ++tick) policy.Step(observation, {.3f, 0, 0});
    Require(policy.SelectedMode() == Mode::Parkour, "Warmup fixture never reached Parkour");
    policy.WarmUp(12);
    Require(policy.SelectedMode() == Mode::Dwaq, "Warmup retained the previous mode");
    // Warmup must discard its synthetic observations, actions and pending forward confirmation.
    for (const auto command : {std::array<float, 3>{0, 0, 0}, {.3f, 0, 0}, {-.2f, 0, 0}})
        for (int tick = 0; tick < 30; ++tick)
            Require(Equal(policy.Step(observation, command), reference.Step(observation, command)),
                    "Warmup polluted real histories or switch timing");
    bool failed = false;
    try { policy.WarmUp(0); } catch (const std::exception&) { failed = true; }
    Require(failed, "Zero warmup steps accepted");
}
}  // namespace
int main() {
    try {
        CheckSelector(); CheckDwaqTrainingHistory(); CheckDwaqUnchanged(); CheckSwitchesAndReset(); CheckWarmupDoesNotLeakHistory();
        std::cout << "Hybrid selection, histories, gains and both real-model transitions passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
