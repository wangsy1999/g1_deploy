#pragma once

#include "g1_hardware.h"
#include <unitree/robot/channel/channel_publisher.hpp>

namespace g1_control {
using MonitorCommand = unitree_hg::msg::dds_::LowCmd_;
inline MonitorCommand EnableStatus(bool enabled) {
    MonitorCommand status{};
    for (int joint = 0; joint < 29; ++joint) status.motor_cmd()[joint].mode() = enabled ? 1 : 0;
    status.crc() = g1_hardware::Crc(status);
    return status;
}
inline MonitorCommand PreviewTarget(const MonitorCommand& command) {
    auto preview = command;
    for (auto& motor : preview.motor_cmd()) motor.mode() = 0;
    preview.crc() = g1_hardware::Crc(preview);
    return preview;
}

// Telemetry only. This class never creates a motor-command publisher.
class MonitorPublisher {
public:
    MonitorPublisher() : targets_("rt/g1/reset_target"), previews_("rt/g1/dry_run_target"),
                         enables_("rt/g1/reset_enable") {}
    void Start() {
        targets_.InitChannel(); previews_.InitChannel(); enables_.InitChannel();
    }
    void PublishEnable(bool enabled) { enables_.Write(EnableStatus(enabled)); }
    void PublishTarget(const MonitorCommand& command) {
        auto target = command;
        target.crc() = g1_hardware::Crc(target);
        targets_.Write(target);
    }
    void PublishPreview(const MonitorCommand& command) { previews_.Write(PreviewTarget(command)); }
private:
    unitree::robot::ChannelPublisher<MonitorCommand> targets_, previews_, enables_;
};
}  // namespace g1_control
