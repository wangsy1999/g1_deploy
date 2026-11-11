#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_onnx {
inline Ort::SessionOptions CpuSessionOptions() {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    return options;
}

inline std::vector<int64_t> TensorShape(Ort::Session& session, bool input, std::size_t index) {
    const auto type = input ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
    const auto info = type.GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error("Expected float32 ONNX tensor");
    return info.GetShape();
}
inline std::string TensorName(Ort::Session& session, bool input, std::size_t index) {
    Ort::AllocatorWithDefaultOptions allocator;
    auto name = input ? session.GetInputNameAllocated(index, allocator)
                      : session.GetOutputNameAllocated(index, allocator);
    return name.get();
}
inline void ValidateSingleInputNetwork(Ort::Session& session,
                                       const std::vector<int64_t>& input_shape,
                                       const std::vector<int64_t>& output_shape,
                                       std::string& input_name, std::string& output_name) {
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
        throw std::runtime_error("Expected single-input single-output network");
    if (TensorShape(session, true, 0) != input_shape || TensorShape(session, false, 0) != output_shape)
        throw std::runtime_error("Incompatible ONNX tensor shape");
    input_name = TensorName(session, true, 0);
    output_name = TensorName(session, false, 0);
}

struct InputTensor {
    const char* name;
    float* data;
    std::size_t size;
    std::vector<int64_t> shape;
};

// Buffers and names are borrowed for this synchronous call only.
template<std::size_t OutputSize>
std::array<float, OutputSize> Run(Ort::Session& session, const std::vector<InputTensor>& inputs,
                                   const std::string& output_name) {
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> tensors;
    std::vector<const char*> names;
    tensors.reserve(inputs.size());
    names.reserve(inputs.size());
    for (const auto& input : inputs) {
        tensors.push_back(Ort::Value::CreateTensor<float>(memory, input.data, input.size,
                                                         input.shape.data(), input.shape.size()));
        names.push_back(input.name);
    }
    const char* output = output_name.c_str();
    auto result = session.Run(Ort::RunOptions{nullptr}, names.data(), tensors.data(), tensors.size(), &output, 1);
    const auto info = result.front().GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || info.GetElementCount() != OutputSize)
        throw std::runtime_error("Unexpected ONNX output type/size");
    const auto* values = result.front().GetTensorData<float>();
    std::array<float, OutputSize> output_values{};
    for (std::size_t i = 0; i < OutputSize; ++i) {
        if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite network output");
        output_values[i] = values[i];
    }
    return output_values;
}

// Convenience overload for a single float-vector input.
template<std::size_t OutputSize>
std::array<float, OutputSize> Run(Ort::Session& session, std::vector<float>& data,
                                   const std::vector<int64_t>& shape,
                                   const std::string& input_name, const std::string& output_name) {
    return Run<OutputSize>(session, {{input_name.c_str(), data.data(), data.size(), shape}}, output_name);
}
}  // namespace g1_onnx
