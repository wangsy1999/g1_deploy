#pragma once
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <atomic>

namespace g1_control {
using MotorCommand=unitree_hg::msg::dds_::LowCmd_;
class MotorPublisher {
    unitree::robot::ChannelPtr<MotorCommand> channel_;
public:
    explicit MotorPublisher(const std::string& name):name_(name) {}
    void InitChannel() {channel_=unitree::robot::ChannelFactory::Instance()->CreateSendChannel<MotorCommand>(name_);}
    bool Write(const MotorCommand& command) {return channel_ && channel_->Write(command,0);}
    uint64_t Identity() const {return channel_->GetWriter()->GetNative().instance_handle().delegate().handle();}
private:
    std::string name_;
};

// Sample metadata identifies the writer, even when two writers send identical
// damping commands. Never modify reserved motor-command fields for identification.
class CommandMonitor: public dds::sub::NoOpDataReaderListener<MotorCommand> {
    std::atomic<bool> armed_{false};
    std::atomic<uint64_t> self_{0};
    std::atomic<bool> conflict_{false};
    unitree::robot::ChannelPtr<MotorCommand> channel_;
    void on_data_available(dds::sub::DataReader<MotorCommand>& reader) override {
        const bool armed=armed_.load();
        const auto self=self_.load();
        try {
            for(const auto& sample:reader.take())
                if(armed && sample.info().valid() &&
                   sample.info().publication_handle().delegate().handle()!=self) conflict_=true;
        } catch(...) {if(armed) conflict_=true;}
    }
public:
    CommandMonitor() {
        channel_=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<MotorCommand>("rt/lowcmd",[](const void*) {},0);
        auto reader=channel_->GetReader()->GetNative();
        reader.listener(this,dds::core::status::StatusMask::data_available());
    }
    ~CommandMonitor() {
        auto reader=channel_->GetReader()->GetNative();
        reader.listener(nullptr,dds::core::status::StatusMask::none());
        channel_.reset();
    }
    void Arm(const MotorPublisher& publisher) {
        armed_=false;
        // Discard pre-handover traffic. A callback already in progress snapshots
        // the disarmed state, so it cannot classify old samples after arming.
        auto reader=channel_->GetReader()->GetNative();
        reader.take();
        self_=publisher.Identity();conflict_=false;armed_=true;
    }
    bool Conflict() const {return conflict_.load();}
};
} // namespace g1_control
