#pragma once
#include "g1_gamepad.h"
#include <cstdint>

namespace g1_unitree_remote {
// Official G1 gamepad.hpp wire order: header[2], keys LE16, lx/rx/ry/L2/ly float32.
// Built-in remote uses direct stick velocity after policy entry, without an R1 gate.
class Input {
    bool initialized_ = false;
public:
    g1_gamepad::State state{true,false};
    void Update(const std::array<uint8_t,40>& raw) {
        std::array<float,5> axes{};
        for (int i=0;i<5;++i) {
            const auto* p=raw.data()+4+i*4;
            const uint32_t bits=uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
            std::memcpy(&axes[i],&bits,sizeof(float));
            if (!std::isfinite(axes[i]) || std::abs(axes[i])>1.05f)
                throw std::runtime_error("Invalid Unitree remote axis; stopping");
        }
        auto js=[](float v) {return static_cast<short>(std::lround(std::clamp(v,-1.f,1.f)*32767.f));};
        state.Axis(ABS_X,js(axes[0]));
        state.Axis(ABS_Y,js(-axes[4]));
        state.Axis(ABS_RX,js(axes[1]));
        const unsigned keys=unsigned(raw[2]) | unsigned(raw[3])<<8;
        const bool initial=!initialized_;
        state.Button(BTN_START,keys&(1u<<2),initial); // Start = real handover
        state.Button(BTN_A,keys&(1u<<10),initial);   // X = reset
        state.Button(BTN_Y,keys&(1u<<8),initial);   // A = policy
        // A held Select must stop even in the first received packet.
        state.Button(BTN_B,keys&(1u<<3),false);     // Select = stop/exit
        initialized_=true;
    }
};
} // namespace g1_unitree_remote
