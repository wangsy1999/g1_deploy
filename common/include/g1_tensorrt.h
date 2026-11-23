#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace g1_tensorrt {
using Shape = std::vector<int64_t>;

struct Input {
    const float* data;
    std::size_t size;
};

// One fixed-shape FP32 network. Calls on a given instance must be serialized.
class Network {
public:
    explicit Network(const std::string& model_path);
    ~Network();

    const std::vector<Shape>& InputShapes() const;
    const Shape& OutputShape() const;
    void ValidateShapes(const Shape& input, const Shape& output) const;

    template<std::size_t OutputSize>
    std::array<float, OutputSize> Run(const Input* inputs, std::size_t count) {
        std::array<float, OutputSize> output{};
        Execute(inputs, count, output.data(), output.size());
        return output;
    }

    template<std::size_t OutputSize>
    std::array<float, OutputSize> Run(const std::vector<float>& data) {
        const Input input{data.data(), data.size()};
        return Run<OutputSize>(&input, 1);
    }

private:
    void Execute(const Input* inputs, std::size_t count, float* output, std::size_t output_size);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace g1_tensorrt
