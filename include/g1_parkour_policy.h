#pragma once
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_parkour {
using Joints = std::array<float, 29>;
using Image = std::array<float, 18*32>;
struct Observation {
    uint64_t sequence = 0;
    double time = 0;
    Joints q{}, dq{}; // policy joint order
    std::array<float,3> omega{}, gravity{};
    float height = 0;
    std::array<float,36*64> depth{}; // image-plane metres
};

class Policy {
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "g1_parkour"};
    std::unique_ptr<Ort::Session> encoder_, actor_;
    std::string encoder_in_, encoder_out_, actor_in_, actor_out_;
    std::deque<std::array<float,96>> history_;
    std::deque<Image> images_;
    Joints last_action_{};
    int delay_ = 0;

    template<std::size_t N>
    static std::array<float,N> Read(const YAML::Node& node) {
        if (!node.IsSequence() || node.size()!=N) throw std::runtime_error("Invalid parkour configuration array");
        std::array<float,N> values{};
        for (std::size_t i=0; i<N; ++i) {
            values[i]=node[i].as<float>();
            if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite configuration");
        }
        return values;
    }
    static void Validate(Ort::Session& session, const std::vector<int64_t>& input,
                         const std::vector<int64_t>& output, std::string& in, std::string& out) {
        if (session.GetInputCount()!=1 || session.GetOutputCount()!=1)
            throw std::runtime_error("Expected single-input single-output network");
        const auto input_type = session.GetInputTypeInfo(0);
        const auto output_type = session.GetOutputTypeInfo(0);
        const auto a = input_type.GetTensorTypeAndShapeInfo();
        const auto b = output_type.GetTensorTypeAndShapeInfo();
        if (a.GetShape()!=input || b.GetShape()!=output ||
            a.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            b.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("Incompatible parkour ONNX tensor shape/type");
        Ort::AllocatorWithDefaultOptions allocator;
        in=session.GetInputNameAllocated(0,allocator).get();
        out=session.GetOutputNameAllocated(0,allocator).get();
    }
    static std::vector<float> Run(Ort::Session& session, std::vector<float>& data,
                                  const std::vector<int64_t>& shape,
                                  const std::string& input, const std::string& output) {
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto tensor=Ort::Value::CreateTensor<float>(memory,data.data(),data.size(),shape.data(),shape.size());
        const char* in=input.c_str(); const char* out=output.c_str();
        auto result=session.Run(Ort::RunOptions{nullptr},&in,&tensor,1,&out,1);
        const auto count=result[0].GetTensorTypeAndShapeInfo().GetElementCount();
        const auto* values=result[0].GetTensorData<float>();
        for(std::size_t i=0;i<count;++i)
            if(!std::isfinite(values[i])) throw std::runtime_error("Non-finite network output");
        return {values,values+count};
    }
public:
    std::array<int,29> ids{}; // policy -> DDS
    Joints neutral{}, kp{}, kd{}, scale{}, sign{}; // policy order
    uint32_t signature=2166136261u;
    double dt=.02;

    Policy(const std::string& models, const std::string& config) {
        const auto cfg=YAML::LoadFile(config);
        neutral=Read<29>(cfg["default_joint_pos"]);
        kp=Read<29>(cfg["kp"]); kd=Read<29>(cfg["kd"]); scale=Read<29>(cfg["action_scale"]);
        sign=Read<29>(cfg["joint_sign"]);
        delay_=cfg["depth_delay_frames"].as<int>();
        dt=cfg["step_dt"].as<double>();
        if((delay_!=0 && delay_!=1) || !std::isfinite(dt) || std::abs(dt-.02)>1e-8)
            throw std::runtime_error("Expected 20ms parkour observations and depth delay 0 or 1");
        const std::vector<std::string> dds={
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint",
            "left_ankle_pitch_joint","left_ankle_roll_joint","right_hip_pitch_joint","right_hip_roll_joint",
            "right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint","left_shoulder_pitch_joint",
            "left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint",
            "left_wrist_pitch_joint","left_wrist_yaw_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint",
            "right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"};
        auto names=cfg["joint_names"].as<std::vector<std::string>>();
        if(names.size()!=29) throw std::runtime_error("Expected 29 policy joints");
        std::array<bool,29> seen{};
        for(int i=0;i<29;++i) {
            auto found=std::find(dds.begin(),dds.end(),names[i]);
            if(found==dds.end() || seen[found-dds.begin()]) throw std::runtime_error("Unknown or duplicate joint");
            ids[i]=found-dds.begin(); seen[ids[i]]=true;
            if(sign[i]!=1 && sign[i]!=-1) throw std::runtime_error("joint_sign must be +1 or -1");
            for(unsigned char c:names[i]+":"+std::to_string(static_cast<int>(sign[i]))+"\n") signature=(signature^c)*16777619u;
            if(kp[i]<=0 || kd[i]<0) throw std::runtime_error("Invalid PD gains");
        }
        Ort::SessionOptions options; options.SetIntraOpNumThreads(1); options.SetInterOpNumThreads(1);
        encoder_=std::make_unique<Ort::Session>(env_,(models+"/0-depth_encoder.onnx").c_str(),options);
        actor_=std::make_unique<Ort::Session>(env_,(models+"/actor.onnx").c_str(),options);
        Validate(*encoder_,{1,8,18,32},{1,128},encoder_in_,encoder_out_);
        Validate(*actor_,{1,896},{1,29},actor_in_,actor_out_);
    }
    void Reset() { history_.clear(); images_.clear(); last_action_.fill(0); }
    static Image Preprocess(const std::array<float,36*64>& depth) {
        for(float v:depth) if(std::isnan(v) || v<0) throw std::runtime_error("Invalid raw depth");
        Image image{};
        const double w0=std::exp(-.5), sum=1+2*w0;
        const std::array<double,3> weights{w0/sum,1/sum,w0/sum};
        auto reflect=[](int v,int size) {return v<0 ? -v : (v>=size ? 2*size-2-v : v);};
        for(int y=0;y<18;++y) for(int x=0;x<32;++x) {
            double value=0;
            for(int j=-1;j<=1;++j) for(int i=-1;i<=1;++i) {
                float d=depth[(reflect(y+j,18)+18)*64+reflect(x+i,32)+16];
                // An infinite no-hit remains far after blur and depth clipping.
                value+=weights[j+1]*weights[i+1]*(std::isinf(d)?1e30:double(d));
            }
            image[y*32+x]=std::clamp(value,0.,2.5)/2.5;
        }
        return image;
    }
    std::pair<std::vector<float>,std::vector<float>> Observe(const Observation& obs,
                                                           const std::array<float,3>& command) {
        std::array<float,96> frame{};
        for(int i=0;i<3;++i) {frame[i]=obs.omega[i]*.25f;frame[3+i]=obs.gravity[i];frame[6+i]=command[i];}
        for(int i=0;i<29;++i) {frame[9+i]=obs.q[i]-neutral[i];frame[38+i]=obs.dq[i]*.05f;frame[67+i]=last_action_[i];}
        for(float v:frame) if(!std::isfinite(v)) throw std::runtime_error("Non-finite proprioception");
        const auto image=Preprocess(obs.depth);
        if(history_.empty()) history_.assign(8,frame); else {history_.pop_front();history_.push_back(frame);}
        if(images_.empty()) images_.assign(37,image); else {images_.pop_front();images_.push_back(image);}
        std::vector<float> proprio, depth;
        proprio.reserve(768); depth.reserve(8*18*32);
        const std::array<int,6> starts{0,3,6,9,38,67}, sizes{3,3,3,29,29,29};
        for(int term=0;term<6;++term) for(const auto& f:history_)
            proprio.insert(proprio.end(),f.begin()+starts[term],f.begin()+starts[term]+sizes[term]);
        for(int offset=35;offset>=0;offset-=5) {
            const auto& im=images_[36-delay_-offset]; depth.insert(depth.end(),im.begin(),im.end());
        }
        return {proprio,depth};
    }
    Joints Step(const Observation& obs,const std::array<float,3>& command) {
        auto inputs=Observe(obs,command);
        auto latent=Run(*encoder_,inputs.second,{1,8,18,32},encoder_in_,encoder_out_);
        inputs.first.insert(inputs.first.end(),latent.begin(),latent.end());
        auto actions=Run(*actor_,inputs.first,{1,896},actor_in_,actor_out_);
        Joints target{};
        for(int i=0;i<29;++i) {
            last_action_[i]=actions[i]; target[ids[i]]=sign[i]*(neutral[i]+scale[i]*actions[i]);
            if(!std::isfinite(target[ids[i]])) throw std::runtime_error("Non-finite target");
        }
        return target;
    }
    Joints DefaultDds() const {Joints result{};for(int i=0;i<29;++i) result[ids[i]]=sign[i]*neutral[i];return result;}
};

// Local UDP protocol: little endian, 32-byte header and 2369 float32 values.
inline Observation Decode(const unsigned char* bytes,std::size_t size,uint32_t signature) {
    constexpr std::size_t count=29+29+3+3+1+36*64;
    if(size!=32+count*4 || std::memcmp(bytes,"G1PKR001",8)!=0) throw std::runtime_error("Invalid observation packet");
    uint32_t order=0,n=0; Observation result;
    std::memcpy(&result.sequence,bytes+8,8);std::memcpy(&result.time,bytes+16,8);
    std::memcpy(&order,bytes+24,4);std::memcpy(&n,bytes+28,4);
    if(order!=signature || n!=count || !std::isfinite(result.time)) throw std::runtime_error("Observation joint order/count/time mismatch");
    std::array<float,count> values{};std::memcpy(values.data(),bytes+32,count*4);
    std::copy_n(values.begin(),29,result.q.begin());std::copy_n(values.begin()+29,29,result.dq.begin());
    std::copy_n(values.begin()+58,3,result.omega.begin());std::copy_n(values.begin()+61,3,result.gravity.begin());
    result.height=values[64];std::copy_n(values.begin()+65,36*64,result.depth.begin());
    for(int i=0;i<65;++i) if(!std::isfinite(values[i])) throw std::runtime_error("Non-finite observation state");
    for(float d:result.depth) if(std::isnan(d)||d<0) throw std::runtime_error("Invalid observation depth");
    return result;
}
} // namespace g1_parkour
