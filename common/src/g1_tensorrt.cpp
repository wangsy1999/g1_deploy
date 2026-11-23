#include "g1_tensorrt.h"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#if NV_TENSORRT_MAJOR < 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR < 5) || NV_TENSORRT_MAJOR > 10
#error "g1_deploy requires TensorRT 8.5 through 10.x"
#endif

namespace g1_tensorrt {
namespace {
class Logger final : public nvinfer1::ILogger {
    void log(Severity severity, const char* message) noexcept override {
        if (severity <= Severity::kWARNING) std::fprintf(stderr, "TensorRT: %s\n", message);
    }
};

// TensorRT uses a process-wide logger for its runtime and builder objects.
Logger logger;

void CheckCuda(cudaError_t status, const char* operation) {
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
    }
}

template<typename T>
std::unique_ptr<T> Require(T* object, const std::string& error) {
    if (!object) throw std::runtime_error(error);
    return std::unique_ptr<T>(object);
}

std::unique_ptr<nvinfer1::IHostMemory> BuildEngine(const std::string& path) {
    auto builder = Require(nvinfer1::createInferBuilder(logger), "Cannot create TensorRT builder");
    uint32_t flags = 0;
#if NV_TENSORRT_MAJOR < 10
    flags = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#endif
    auto network = Require(builder->createNetworkV2(flags), "Cannot create TensorRT network");
    auto parser = Require(nvonnxparser::createParser(*network, logger), "Cannot create TensorRT ONNX parser");
    if (!parser->parseFromFile(path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        std::string error = "Cannot parse ONNX model";
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            error += "\n";
            error += parser->getError(i)->desc();
        }
        throw std::runtime_error(error);
    }
    auto config = Require(builder->createBuilderConfig(), "Cannot create TensorRT builder config");
    config->clearFlag(nvinfer1::BuilderFlag::kTF32);
    std::fprintf(stderr, "Building TensorRT engine for %s\n", path.c_str());
    return Require(builder->buildSerializedNetwork(*network, *config), "Cannot build TensorRT engine");
}

struct Tensor {
    std::size_t size = 0;
    std::unique_ptr<void, decltype(&cudaFree)> device{nullptr, cudaFree};
    std::unique_ptr<float, decltype(&cudaFreeHost)> host{nullptr, cudaFreeHost};
};

Tensor BindTensor(nvinfer1::IExecutionContext& context, const char* name, const Shape& shape) {
    Tensor tensor;
    tensor.size = 1;
    for (const auto dim : shape) {
        if (dim <= 0 || static_cast<uint64_t>(dim) >
                std::numeric_limits<std::size_t>::max() / sizeof(float) / tensor.size) {
            throw std::runtime_error(std::string("Expected a fixed tensor shape for ") + name);
        }
        tensor.size *= static_cast<std::size_t>(dim);
    }
    void* device = nullptr;
    CheckCuda(cudaMalloc(&device, tensor.size * sizeof(float)), "cudaMalloc");
    tensor.device.reset(device);
    void* host = nullptr;
    CheckCuda(cudaMallocHost(&host, tensor.size * sizeof(float)), "cudaMallocHost");
    tensor.host.reset(static_cast<float*>(host));
    if (!context.setTensorAddress(name, tensor.device.get())) {
        throw std::runtime_error(std::string("Cannot bind TensorRT tensor ") + name);
    }
    return tensor;
}
}  // namespace

struct Network::Impl {
    std::string model_path;
    int device = 0;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    std::unique_ptr<std::remove_pointer_t<cudaStream_t>, decltype(&cudaStreamDestroy)> stream{
        nullptr, cudaStreamDestroy};
    std::vector<Tensor> inputs;
    Tensor output;
    std::vector<Shape> input_shapes;
    Shape output_shape;
};

Network::Network(const std::string& model_path) : impl_(std::make_unique<Impl>()) {
    auto& model = *impl_;
    model.model_path = model_path;
    try {
        CheckCuda(cudaGetDevice(&model.device), "cudaGetDevice");
        model.runtime = Require(nvinfer1::createInferRuntime(logger), "Cannot create TensorRT runtime");
        const auto extension = std::filesystem::path(model_path).extension();
        if (extension == ".engine" || extension == ".plan") {
            std::ifstream file(model_path, std::ios::binary);
            if (!file) throw std::runtime_error("Cannot open TensorRT engine");
            const std::vector<char> bytes((std::istreambuf_iterator<char>(file)), {});
            if (bytes.empty()) throw std::runtime_error("Empty TensorRT engine");
            model.engine = Require(model.runtime->deserializeCudaEngine(bytes.data(), bytes.size()),
                                   "Cannot deserialize TensorRT engine for this GPU and runtime");
        } else {
            const auto engine = BuildEngine(model_path);
            model.engine = Require(model.runtime->deserializeCudaEngine(engine->data(), engine->size()),
                                   "Cannot deserialize the newly built TensorRT engine");
        }
        model.context = Require(model.engine->createExecutionContext(), "Cannot create TensorRT execution context");
        cudaStream_t stream = nullptr;
        CheckCuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), "cudaStreamCreateWithFlags");
        model.stream.reset(stream);

        for (int i = 0; i < model.engine->getNbIOTensors(); ++i) {
            const char* name = model.engine->getIOTensorName(i);
            if (!name || model.engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT ||
                model.engine->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE ||
                model.engine->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR ||
                model.engine->isShapeInferenceIO(name)) {
                throw std::runtime_error("Expected linear, device-resident FP32 TensorRT input/output tensors");
            }
            const auto dims = model.engine->getTensorShape(name);
            if (dims.nbDims <= 0) throw std::runtime_error(std::string("Invalid tensor rank for ") + name);
            Shape shape(dims.d, dims.d + dims.nbDims);
            if (model.engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
                model.inputs.push_back(BindTensor(*model.context, name, shape));
                model.input_shapes.push_back(std::move(shape));
            } else {
                if (model.output.size != 0) throw std::runtime_error("Expected one TensorRT output");
                model.output = BindTensor(*model.context, name, shape);
                model.output_shape = std::move(shape);
            }
        }
        if (model.inputs.empty() || model.output.size == 0) {
            throw std::runtime_error("Expected at least one input and one output");
        }
    } catch (const std::exception& error) {
        throw std::runtime_error("TensorRT model " + model_path + ": " + error.what());
    }
}

Network::~Network() = default;

const std::vector<Shape>& Network::InputShapes() const { return impl_->input_shapes; }
const Shape& Network::OutputShape() const { return impl_->output_shape; }

void Network::ValidateShapes(const Shape& input, const Shape& output) const {
    if (InputShapes().size() != 1 || InputShapes().front() != input || OutputShape() != output) {
        throw std::runtime_error("Incompatible TensorRT tensor shapes for " + impl_->model_path);
    }
}

void Network::Execute(const Input* inputs, std::size_t count, float* output, std::size_t output_size) {
    auto& model = *impl_;
    if (count != model.inputs.size() || output_size != model.output.size) {
        throw std::runtime_error("Unexpected TensorRT input count or output size");
    }
    for (std::size_t i = 0; i < count; ++i) {
        if (!inputs[i].data || inputs[i].size != model.inputs[i].size) {
            throw std::runtime_error("Unexpected TensorRT input buffer");
        }
        std::copy_n(inputs[i].data, inputs[i].size, model.inputs[i].host.get());
    }

    // Policies are constructed on the main thread and can run on a worker thread.
    CheckCuda(cudaSetDevice(model.device), "cudaSetDevice");
    const auto stream = model.stream.get();
    try {
        for (const auto& tensor : model.inputs) {
            CheckCuda(cudaMemcpyAsync(tensor.device.get(), tensor.host.get(), tensor.size * sizeof(float),
                                      cudaMemcpyHostToDevice, stream), "TensorRT input copy");
        }
        if (!model.context->enqueueV3(stream)) throw std::runtime_error("TensorRT enqueueV3 failed");
        CheckCuda(cudaMemcpyAsync(model.output.host.get(), model.output.device.get(), output_size * sizeof(float),
                                  cudaMemcpyDeviceToHost, stream), "TensorRT output copy");
        CheckCuda(cudaStreamSynchronize(stream), "TensorRT stream synchronization");
    } catch (const std::exception& error) {
        // An earlier copy or kernel may still be using the reusable buffers.
        cudaStreamSynchronize(stream);
        throw std::runtime_error("TensorRT inference for " + model.model_path + ": " + error.what());
    }
    const auto* values = model.output.host.get();
    if (!std::all_of(values, values + output_size, [](float value) { return std::isfinite(value); })) {
        throw std::runtime_error("Non-finite TensorRT output for " + model.model_path);
    }
    std::copy_n(values, output_size, output);
}
}  // namespace g1_tensorrt
