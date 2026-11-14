#include "g1_parkour_policy.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
struct Fixture {
    std::filesystem::path directory = std::filesystem::temp_directory_path() /
        ("g1-action-feedback-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    YAML::Node cfg = YAML::LoadFile(G1_PARKOUR_TEST_CONFIG);
    Fixture() {
        std::filesystem::create_directory(directory);
        for (const char* key : {"actor", "depth_encoder"})
            cfg["models"][key] = g1_config::ResolvePath(G1_PARKOUR_TEST_CONFIG, cfg["models"][key].as<std::string>());
    }
    ~Fixture() { std::filesystem::remove_all(directory); }
    std::string Save() const {
        const auto path = directory / "policy.yaml";
        std::ofstream file(path);
        file << cfg;
        return path.string();
    }
};

// 使用真实模型配置，检查动作处理到下一帧观测的完整链路。
void CheckFeedback(Fixture& fixture, float expected_action) {
    g1_parkour::Policy policy(fixture.Save());
    g1_parkour::Joints actions{};
    for (int joint = 0; joint < 29; ++joint) actions[joint] = joint % 2 ? -1000.f : 1000.f;
    actions[2] = .5f;
    const auto targets = policy.ProcessActions(actions);
    g1_parkour::Observation obs;
    obs.q = policy.neutral;
    obs.gravity = {0, 0, -1};
    obs.depth.fill(2.5f);
    const auto input = policy.Observe(obs, {0, 0, 0});
    constexpr int action_offset = (3 + 3 + 3 + 29 + 29) * 8;
    for (int joint = 0; joint < 29; ++joint) {
        const float feedback = joint == 2 ? .5f : (joint % 2 ? -expected_action : expected_action);
        for (int frame = 0; frame < 8; ++frame)
            Require(input.proprioception[action_offset + frame * 29 + joint] == feedback,
                    "Parkour last_action feedback differs from configuration");
        const float clip = fixture.cfg["raw_clip"].as<float>(std::numeric_limits<float>::max());
        const float applied = std::clamp(actions[joint], -clip, clip);
        const float angle = policy.sign[joint] * (policy.neutral[joint] + policy.scale[joint] * applied);
        const float expected_target = std::clamp(angle,
            fixture.cfg["joint_clip_lower"][joint].as<float>(), fixture.cfg["joint_clip_upper"][joint].as<float>());
        Require(targets[policy.ids[joint]] == expected_target, "Parkour target conversion changed");
    }
    policy.Reset();
    const auto reset = policy.Observe(obs, {0, 0, 0});
    for (int index = action_offset; index < 768; ++index)
        Require(reset.proprioception[index] == 0, "Reset retained action feedback");
    actions[0] = std::numeric_limits<float>::quiet_NaN();
    bool rejected = false;
    try { policy.ProcessActions(actions); } catch (const std::runtime_error&) { rejected = true; }
    Require(rejected, "Non-finite action accepted");
}
} // namespace
int main() {
    try {
        Fixture fixture;
        CheckFeedback(fixture, 100.f);
        fixture.cfg["clip_action_feedback"] = false;
        CheckFeedback(fixture, 1000.f);
        fixture.cfg.remove("raw_clip");
        fixture.cfg.remove("clip_action_feedback");
        CheckFeedback(fixture, 1000.f);
        for (const char* invalid : {"0", "-1", ".nan", ".inf"}) {
            fixture.cfg["raw_clip"] = YAML::Load(invalid);
            bool rejected = false;
            try { g1_parkour::Policy policy(fixture.Save()); }
            catch (const std::runtime_error&) { rejected = true; }
            Require(rejected, "Invalid raw clip accepted");
        }
        std::cout << "Parkour clipping, feedback, reset and validation checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
