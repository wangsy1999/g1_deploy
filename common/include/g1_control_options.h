#pragma once

#include <cmath>
#include <net/if.h>
#include <stdexcept>
#include <string>

namespace g1_control {
struct RuntimeOptions {
    bool hardware = false;
    double reset_seconds = 3;
    int domain = 1;
    std::string interface = "lo";
    std::string config;
};
struct OptionState {
    bool mode_set = false;
    bool named_values = false;
};
inline std::string OptionValue(int argc, char** argv, int& index, const std::string& option) {
    if (index + 1 == argc) throw std::invalid_argument("Missing value for " + option);
    return argv[++index];
}
inline int ParseInteger(const std::string& text) {
    std::size_t consumed = 0;
    const int result = std::stoi(text, &consumed);
    if (consumed != text.size()) throw std::invalid_argument("Invalid integer: " + text);
    return result;
}
inline double ParseNumber(const std::string& text) {
    std::size_t consumed = 0;
    const double result = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(result))
        throw std::invalid_argument("Invalid number: " + text);
    return result;
}
inline void SelectMode(bool hardware, RuntimeOptions& options, OptionState& state) {
    if (state.mode_set && hardware != options.hardware)
        throw std::invalid_argument("Hardware and simulation modes cannot be combined");
    options.hardware = hardware;
    state.mode_set = true;
}
inline bool ParseRuntimeOption(int argc, char** argv, int& index,
                                RuntimeOptions& options, OptionState& state) {
    const std::string argument = argv[index];
    if (argument == "--hardware" || argument == "--sim") {
        SelectMode(argument == "--hardware", options, state);
        return true;
    }
    if (argument == "--interface") options.interface = OptionValue(argc, argv, index, argument);
    else if (argument == "--domain") options.domain = ParseInteger(OptionValue(argc, argv, index, argument));
    else if (argument == "--config") options.config = OptionValue(argc, argv, index, argument);
    else if (argument == "--reset-seconds") options.reset_seconds = ParseNumber(OptionValue(argc, argv, index, argument));
    else return false;
    state.named_values = true;
    return true;
}
inline void ValidateRuntimeOptions(const RuntimeOptions& options) {
    if (!std::isfinite(options.reset_seconds) || options.reset_seconds <= 0)
        throw std::invalid_argument("reset_seconds must be finite and positive");
    if (options.hardware ? options.domain != 0 : options.domain <= 0)
        throw std::invalid_argument("Hardware domain must be 0; simulation domain must be positive");
    if (options.hardware && (options.interface == "lo" || !if_nametoindex(options.interface.c_str())))
        throw std::invalid_argument("Specify an existing non-loopback robot network interface");
}
}  // namespace g1_control
