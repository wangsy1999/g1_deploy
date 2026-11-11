#pragma once

#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace g1_config {
template<class T, std::size_t N>
std::array<T, N> ReadArray(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() != N)
        throw std::runtime_error("Invalid configuration array length; expected " + std::to_string(N));
    std::array<T, N> values{};
    for (std::size_t i = 0; i < N; ++i) {
        values[i] = node[i].as<T>();
        if (!std::isfinite(static_cast<double>(values[i])))
            throw std::runtime_error("Non-finite configuration array value");
    }
    return values;
}

// Absolute values are retained; relative values are based on the configuration file.
inline std::string ResolvePath(const std::string& config, const std::string& value) {
    return (std::filesystem::path(config).parent_path() / value).lexically_normal().string();
}
}  // namespace g1_config
