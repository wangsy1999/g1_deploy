#pragma once
#include "g1_parkour_policy.h"
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>

namespace g1_parkour {
struct DepthFrame {
    uint64_t sequence = 0;
    double captured_at = 0; // CLOCK_MONOTONIC, same Linux host as the controller
    std::array<float,36*64> image{};
};
inline DepthFrame DecodeDepth(const unsigned char* bytes, std::size_t size) {
    if (size != 32+36*64*4 || std::memcmp(bytes,"G1DEP001",8))
        throw std::runtime_error("Invalid RealSense depth packet");
    DepthFrame frame;
    uint32_t width=0,height=0;
    std::memcpy(&frame.sequence,bytes+8,8);
    std::memcpy(&frame.captured_at,bytes+16,8);
    std::memcpy(&width,bytes+24,4); std::memcpy(&height,bytes+28,4);
    if (width!=64 || height!=36 || !std::isfinite(frame.captured_at))
        throw std::runtime_error("RealSense dimensions/timestamp mismatch");
    std::memcpy(frame.image.data(),bytes+32,frame.image.size()*4);
    for (float d : frame.image)
        if (std::isnan(d) || d<0) throw std::runtime_error("Invalid RealSense depth");
    return frame;
}

class UdpInput {
    int fd_=-1;
public:
    explicit UdpInput(int port) {
        fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
        if(fd_<0) throw std::runtime_error("Cannot create input socket");
        sockaddr_in address{}; address.sin_family=AF_INET;
        address.sin_port=htons(port); address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))) {
            close(fd_); throw std::runtime_error("Cannot bind loopback input port; another controller running?");
        }
    }
    ~UdpInput() {if(fd_>=0) close(fd_);}
    UdpInput(const UdpInput&)=delete;
    UdpInput& operator=(const UdpInput&)=delete;
    template<class Decode> bool Latest(Decode decode) {
        bool fresh=false;
        std::array<unsigned char,65536> packet{};
        for(int i=0;i<64;++i) {
            auto n=recv(fd_,packet.data(),packet.size(),0);
            if(n<0) {
                if(errno==EAGAIN || errno==EWOULDBLOCK) break;
                if(errno==EINTR) continue;
                throw std::runtime_error("Input receive failed");
            }
            decode(packet.data(),static_cast<std::size_t>(n)); fresh=true;
        }
        return fresh;
    }
};

// The policy root is torso. LowState's pelvis IMU is deliberately not used here.
inline Observation HardwareObservation(const Policy& policy,
    const unitree_hg::msg::dds_::LowState_& state,
    const unitree_hg::msg::dds_::IMUState_& torso, const DepthFrame& depth) {
    Observation obs;
    obs.depth=depth.image; obs.sequence=depth.sequence; obs.time=depth.captured_at;
    for(int i=0;i<29;++i) {
        obs.q[i]=policy.sign[i]*state.motor_state()[policy.ids[i]].q();
        obs.dq[i]=policy.sign[i]*state.motor_state()[policy.ids[i]].dq();
    }
    const auto& q=torso.quaternion();
    double norm=0; for(float v:q) norm+=double(v)*v;
    if(!std::isfinite(norm) || norm<.5 || norm>1.5)
        throw std::runtime_error("Invalid torso IMU quaternion");
    obs.gravity={float(2*(double(q[0])*q[2]-double(q[1])*q[3])/norm),
                 float(-2*(double(q[0])*q[1]+double(q[2])*q[3])/norm),
                 float((2*(double(q[1])*q[1]+double(q[2])*q[2])-norm)/norm)};
    std::copy_n(torso.gyroscope().begin(),3,obs.omega.begin());
    for(float v:obs.omega) if(!std::isfinite(v)) throw std::runtime_error("Invalid torso gyro");
    // Absolute torso height is not observable from this IMU/LowState input.
    // Only simulation enables the absolute-height protection.
    obs.height=0;
    return obs;
}
} // namespace g1_parkour
