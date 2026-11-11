#include "g1_config_utils.h"
#include "g1_observation_history.h"
#include "g1_onnx_utils.h"
#include <iostream>
#include <limits>

namespace {
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class Function> void MustFail(Function function) {
    bool failed = false;
    try { function(); } catch (const std::exception&) { failed = true; }
    Require(failed, "Invalid input was accepted");
}
void CheckConfiguration() {
    const auto node = YAML::Load("[1, 2, 3]");
    Require(g1_config::ReadArray<float, 3>(node) == std::array<float, 3>{1, 2, 3}, "Array values changed");
    MustFail([&] { g1_config::ReadArray<float, 2>(node); });
    MustFail([] { g1_config::ReadArray<float, 1>(YAML::Load("[.nan]")); });
    MustFail([] { g1_config::ReadArray<float, 1>(YAML::Load("[.inf]")); });
    MustFail([] { g1_config::ReadArray<int, 1>(YAML::Load("invalid")); });
    Require(g1_config::ResolvePath("/app/config/policy.json", "../models/policy.onnx") == "/app/models/policy.onnx",
            "Model path depends on working directory");
    Require(g1_config::ResolvePath("/app/config/policy.json", "/models/policy.onnx") == "/models/policy.onnx",
            "Absolute model path was rewritten");
}
void CheckHistory() {
    using Frame = std::array<float, 2>;
    std::array<Frame, 2> fixed{};
    std::deque<Frame> dynamic;
    bool initialized = false;
    for (const Frame frame : {Frame{1, 10}, Frame{2, 20}, Frame{3, 30}}) {
        g1_inference::PushHistory(fixed, frame, initialized);
        g1_inference::PushHistory(dynamic, frame, 2);
        Require(fixed[0] == dynamic[0] && fixed[1] == dynamic[1], "History implementations disagree");
    }
    std::array<float, 4> packed{};
    g1_inference::PackTermMajor(fixed, std::array<int, 2>{1, 1}, packed.begin());
    Require(packed == std::array<float, 4>{2, 3, 20, 30}, "Input history is not term-major, oldest first");
    MustFail([&] { g1_inference::PushHistory(dynamic, Frame{}, 3); });
    MustFail([&] { g1_inference::PackTermMajor(fixed, std::array<int, 1>{3}, packed.begin()); });
    initialized = false;
    dynamic.clear();
    g1_inference::PushHistory(fixed, Frame{7, 8}, initialized);
    g1_inference::PushHistory(dynamic, Frame{7, 8}, 2);
    Require(fixed[0] == Frame{7, 8} && fixed[1] == fixed[0] && dynamic[0] == fixed[0], "Reset retained old history");
}
void CheckNetworks() {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "inference_utils_test");
    auto options = g1_onnx::CpuSessionOptions();
    Ort::Session encoder(env, G1_TEST_ENCODER, options);
    std::string input_name, output_name;
    g1_onnx::ValidateSingleInputNetwork(encoder, {1, 8, 18, 32}, {1, 128}, input_name, output_name);
    std::vector<float> depth(8 * 18 * 32, .5f);
    const auto features = g1_onnx::Run<128>(encoder, depth, {1, 8, 18, 32}, input_name, output_name);
    Require(features == g1_onnx::Run<128>(encoder, depth, {1, 8, 18, 32}, input_name, output_name), "Repeated encoder inference changed");
    MustFail([&] { g1_onnx::ValidateSingleInputNetwork(encoder, {1, 7, 18, 32}, {1, 128}, input_name, output_name); });
    MustFail([&] { g1_onnx::Run<127>(encoder, depth, {1, 8, 18, 32}, input_name, output_name); });
    Ort::Session policy(env, G1_TEST_DWAQ_MODEL, options);
    std::vector<std::vector<float>> buffers;
    std::vector<std::string> names;
    std::vector<std::vector<int64_t>> shapes;
    for (std::size_t i = 0; i < policy.GetInputCount(); ++i) {
        auto shape = g1_onnx::TensorShape(policy, true, i);
        Require(shape.size() == 2 && shape[0] == 1, "Unexpected test policy input");
        buffers.emplace_back(static_cast<std::size_t>(shape[1]), 0.f);
        names.push_back(g1_onnx::TensorName(policy, true, i));
        shapes.push_back(shape);
    }
    std::vector<g1_onnx::InputTensor> inputs;
    for (std::size_t i = 0; i < buffers.size(); ++i)
        inputs.push_back({names[i].c_str(), buffers[i].data(), buffers[i].size(), shapes[i]});
    const auto output = g1_onnx::TensorName(policy, false, 0);
    const auto actions = g1_onnx::Run<29>(policy, inputs, output);
    for (float value : actions) Require(std::isfinite(value), "Non-finite policy output");
    MustFail([&] { g1_onnx::ValidateSingleInputNetwork(policy, {1, 480}, {1, 29}, input_name, output_name); });
}
}  // namespace
int main() {
    try {
        CheckConfiguration(); CheckHistory(); CheckNetworks();
        std::cout << "Shared configuration, histories, single/multi-input ONNX checks passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
