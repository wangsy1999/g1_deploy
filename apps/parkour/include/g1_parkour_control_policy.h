#pragma once

#include "g1_parkour_policy.h"
#include <unitree/idl/hg/LowState_.hpp>

namespace g1_parkour_app {
// Camera observations retain their original layout. LowState also preserves the
// pelvis IMU and motor order needed by other locomotion policies.
struct ControlObservation : g1_parkour::Observation {
    unitree_hg::msg::dds_::LowState_ lowstate;
};
struct MotorTargets {
    g1_parkour::Joints position{}, kp{}, kd{}; // all in DDS motor order
};

// A single control loop owns DDS/output/input. Policies only produce targets.
class ControlPolicy {
public:
    virtual ~ControlPolicy() = default;
    virtual const char* Name() const = 0;
    virtual const g1_parkour::Policy& CameraPolicy() const = 0;
    virtual MotorTargets DefaultTargets() const = 0;
    virtual void ClampVelocity(std::array<float, 3>& velocity) const = 0;
    virtual MotorTargets Step(const ControlObservation& observation, const std::array<float, 3>& velocity) = 0;
    virtual void Reset() = 0;
};
}  // namespace g1_parkour_app
