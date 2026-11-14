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
// 控制循环使用单线程 CPU 推理，避免算子线程池与控制线程争抢 CPU。
inline Ort::SessionOptions CpuSessionOptions() {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    return options;
}

// input 为 true 时读取输入，否则读取输出；仅接受策略使用的 float32 张量。
inline std::vector<int64_t> TensorShape(Ort::Session& session, bool input, std::size_t index) {
    const auto type_info = input ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
    const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        throw std::runtime_error("Expected float32 ONNX tensor");
    }
    return tensor_info.GetShape();
}
inline std::string TensorName(Ort::Session& session, bool input, std::size_t index) {
    Ort::AllocatorWithDefaultOptions allocator;
    auto name = input ? session.GetInputNameAllocated(index, allocator)
                      : session.GetOutputNameAllocated(index, allocator);
    // 复制名称，避免返回指向 ONNX 临时分配内存的指针。
    return name.get();
}
inline void ValidateSingleInputNetwork(Ort::Session& session,
                                       const std::vector<int64_t>& input_shape,
                                       const std::vector<int64_t>& output_shape,
                                       std::string& input_name, std::string& output_name) {
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
        throw std::runtime_error("Expected single-input single-output network");
    }
    if (TensorShape(session, true, 0) != input_shape || TensorShape(session, false, 0) != output_shape) {
        throw std::runtime_error("Incompatible ONNX tensor shape");
    }
    input_name = TensorName(session, true, 0);
    output_name = TensorName(session, false, 0);
}

// 输入描述借用名称和数据缓冲区；调用方必须保证它们在 Run 返回前有效。
struct InputTensor {
    const char* name;            // 模型声明的输入名
    float* data;                // 连续 float32 数据，ONNX 接口要求可写指针
    std::size_t size;           // 元素数量，不是字节数
    std::vector<int64_t> shape; // 包含 batch 维度，例如 [1, 480]
};

// 同步执行：借用输入缓冲区创建张量，返回前将输出复制到独立数组。
// OutputSize 是策略期望的输出元素数，运行后还会检查类型、数量及有限性。
template<std::size_t OutputSize>
std::array<float, OutputSize> Run(Ort::Session& session, const std::vector<InputTensor>& inputs,
                                   const std::string& output_name) {
    auto cpu_memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> input_tensors;
    std::vector<const char*> input_names;
    input_tensors.reserve(inputs.size());
    input_names.reserve(inputs.size());
    for (const auto& input : inputs) {
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            cpu_memory, input.data, input.size, input.shape.data(), input.shape.size()));
        input_names.push_back(input.name);
    }
    const char* output_name_pointer = output_name.c_str();
    auto output_tensors = session.Run(
        Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(),
        &output_name_pointer, 1);
    const auto output_info = output_tensors.front().GetTensorTypeAndShapeInfo();
    if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output_info.GetElementCount() != OutputSize) {
        throw std::runtime_error("Unexpected ONNX output type/size");
    }
    const auto* values = output_tensors.front().GetTensorData<float>();
    std::array<float, OutputSize> output_values{};
    for (std::size_t i = 0; i < OutputSize; ++i) {
        if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite network output");
        output_values[i] = values[i];
    }
    return output_values;
}

// 单输入网络的便捷入口，复用相同的输出检查。
template<std::size_t OutputSize>
std::array<float, OutputSize> Run(Ort::Session& session, std::vector<float>& data,
                                   const std::vector<int64_t>& shape,
                                   const std::string& input_name, const std::string& output_name) {
    const InputTensor input{input_name.c_str(), data.data(), data.size(), shape};
    return Run<OutputSize>(session, {input}, output_name);
}
}  // namespace g1_onnx
