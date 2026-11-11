#pragma once

#include <cmath>
#include <stdexcept>

namespace g1_hybrid {
enum class Mode { Dwaq, Parkour };
struct SwitchSettings {
    float forward_enter_mps = .15f;
    float forward_exit_mps = .05f;
    double forward_confirm_seconds = .20;

    void Validate() const {
        if (!std::isfinite(forward_enter_mps) || !std::isfinite(forward_exit_mps) ||
            !std::isfinite(forward_confirm_seconds) ||
            forward_exit_mps < 0 || forward_enter_mps <= forward_exit_mps || forward_enter_mps > 1 ||
            forward_confirm_seconds <= 0)
            throw std::runtime_error("Invalid hybrid switching thresholds or durations");
    }
};

class ModeSelector {
public:
    explicit ModeSelector(SwitchSettings settings) : settings_(settings) { settings_.Validate(); }
    void Reset() { mode_ = Mode::Dwaq; forward_seconds_ = 0; }
    Mode Update(float requested_forward_speed, double dt) {
        if (!std::isfinite(requested_forward_speed) || !std::isfinite(dt) || dt <= 0)
            throw std::runtime_error("Invalid hybrid command or timestep");
        if (mode_ == Mode::Parkour) {
            // 松杆、后退或很小的前进指令，立即选择 DWaQ。
            if (requested_forward_speed <= settings_.forward_exit_mps) Reset();
        } else {
            // 前推必须连续保持一段时间；阈值间的滞回区避免摇杆抖动反复切换。
            forward_seconds_ = requested_forward_speed >= settings_.forward_enter_mps ? forward_seconds_ + dt : 0;
            if (forward_seconds_ + 1e-9 >= settings_.forward_confirm_seconds) {
                mode_ = Mode::Parkour;
                forward_seconds_ = 0;
            }
        }
        return mode_;
    }
private:
    SwitchSettings settings_;
    Mode mode_ = Mode::Dwaq;
    double forward_seconds_ = 0;
};

}  // namespace g1_hybrid
