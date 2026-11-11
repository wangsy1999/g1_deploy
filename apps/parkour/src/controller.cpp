// Parkour controller: synchronized observations, reset/hold and actuator lifecycle.
#include "g1_parkour_controller.h"
#include "g1_parkour_control_policy.h"
#include "g1_loop_scheduler.h"
#include "g1_dds_state_cache.h"
#include "g1_monitor_publisher.h"
#include "g1_joint_mapping.h"
#include "g1_keyboard.h"
#include "g1_control_signals.h"
#include "g1_control_runtime.h"
#include "g1_motion_service.h"
#include "g1_control_targets.h"
#include "g1_parkour_input.h"
#include "g1_policy_worker.h"
#include "g1_hardware.h"
#include "g1_motor_channel.h"
#include "g1_async_log.h"
#include "g1_unitree_remote.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

namespace {
using Clock=std::chrono::steady_clock;
using Cmd=unitree_hg::msg::dds_::LowCmd_;
using State=unitree_hg::msg::dds_::LowState_;
using Imu=unitree_hg::msg::dds_::IMUState_;
}  // namespace

namespace g1_parkour_app {
namespace {
int CheckPolicy(g1_parkour::Policy& policy) {
    g1_parkour::Observation obs;
    obs.q = policy.neutral;
    obs.gravity = {0, 0, -1};
    obs.depth.fill(2.5f);
    const auto target = policy.Step(obs, {0, 0, 0});
    std::cout << "CPU parkour double-ONNX inference OK; DDS targets:";
    for (float v : target) std::cout << ' ' << v;
    std::cout << '\n';
    return 0;
}

// Preserve standalone Parkour behavior behind the reusable control-loop interface.
class ParkourPolicy final : public ControlPolicy {
public:
    explicit ParkourPolicy(g1_parkour::Policy& policy) : policy_(policy) {}
    const char* Name() const override { return "Parkour"; }
    const g1_parkour::Policy& CameraPolicy() const override { return policy_; }
    MotorTargets DefaultTargets() const override {
        return {policy_.DefaultDds(), g1_joints::ToMotorOrder(policy_.kp, policy_.ids),
                g1_joints::ToMotorOrder(policy_.kd, policy_.ids)};
    }
    void ClampVelocity(std::array<float, 3>& velocity) const override {
        velocity[0] = std::clamp(velocity[0], 0.f, 1.f);
        velocity[1] = 0;
        velocity[2] = std::clamp(velocity[2], -1.f, 1.f);
    }
    MotorTargets Step(const ControlObservation& observation, const std::array<float, 3>& velocity) override {
        auto result = DefaultTargets();
        result.position = policy_.Step(observation, velocity);
        return result;
    }
    void Reset() override { policy_.Reset(); }
private:
    g1_parkour::Policy& policy_;
};

class ParkourController {
public:
    ParkourController(const Options& options, ControlPolicy& policy, double reset_duration)
        : policy(policy), hardware(options.hardware), config(options.config),
          interface(options.interface), joystick(options.joystick), domain(options.domain), port(options.port),
          reset_seconds(reset_duration), states("rt/lowstate") {}

    int Run() {
        Initialize();
        g1_control::LoopScheduler<> cycle(g1_control::kControlPeriod, waiting);
        int result = 0;
        try {
            while (!g1_control::StopRequested()) {
                const auto now = Clock::now();
                if (!ReadObservation(now)) {
                    std::this_thread::sleep_for(g1_control::kControlPeriod);
                    continue;
                }
                const char key = ReadInput();
                if (key == ' ') break;
                HandleInput(key, now);
                UpdatePolicy(key);
                PublishTargets(now);
                telemetry.PublishEnable(powered);
                cycle.Sleep();
            }
        } catch (const std::exception& error) {
            log.Line() << policy.Name() << " stopped: " << error.what() << '\n';
            result = 1;
        }
        Stop();
        return result;
    }

private:
    void Initialize() {
        if(hardware) {
            limits=std::make_unique<g1_hardware::Limits>(YAML::LoadFile(config));
            const auto pose=policy.DefaultTargets().position;
            for(int i=0;i<29;++i) if(pose[i]<limits->lower[i]+.01f||pose[i]>limits->upper[i]-.01f)
                throw std::runtime_error("Parkour reset pose exceeds hardware limits");
        }
        if(!joystick.empty()) gamepad=std::make_unique<g1_gamepad::Device>(joystick,false,false);
        input=hardware?&remote.state:gamepad?&gamepad->state:nullptr;
        if(hardware) camera=std::make_unique<g1_parkour::DepthReceiver>(port);
        else udp=std::make_unique<g1_parkour::UdpInput>(port);
        unitree::robot::ChannelFactory::Instance()->Init(domain,interface);
        states.Start();
        if(hardware) {
            imus=std::make_unique<g1_control::DdsStateCache<Imu>>("rt/secondary_imu");
            imus->Start();
            monitor=std::make_unique<g1_control::CommandMonitor>();
        }
        telemetry.Start();
        keyboard=std::make_unique<g1_control::Keyboard>();
        command.mode_pr()=0;
        const auto defaults = policy.DefaultTargets();
        neutral = defaults.position;
        motor_kp = defaults.kp;
        motor_kd = defaults.kd;
        policy_worker=std::make_unique<PolicyWorker>(
            [&](const auto& observation,const auto& velocity) {return policy.Step(observation,velocity);},
            [&] {policy.Reset();}, !hardware);
        waiting = Clock::now();
        log.Line()<<policy.Name()<<" source="<<(hardware?"RealSense + torso DDS IMU":"MuJoCo synchronized camera observations")
                 <<"; loopback UDP "<<port<<"; DDS domain "<<domain<<".\n"
                 <<"DRY RUN: p reset, r policy; 9/Start enables real output and clears preview. Space/Select stops.\n";
    }

    bool ReadObservation(Clock::time_point now) {
        if(powered && monitor && monitor->Conflict())
            throw std::runtime_error("Competing LowCmd writer published after handover");
        auto lowstate=states.Read();
        const auto torso=imus ? imus->Read() : g1_control::StateCache<Imu>::Snapshot{};
        snapshot = lowstate.value;
        imu = torso.value;
        const bool state_fresh=lowstate.valid && now-lowstate.received_at<std::chrono::milliseconds(hardware?int(limits->timeout_ms):500);
        const bool torso_fresh=torso.valid && now-torso.received_at<std::chrono::milliseconds(100);
        const double state_age_ms=lowstate.AgeMs(now),torso_age_ms=torso.AgeMs(now);
        if(hardware) {
            camera->Latest([&](const g1_parkour::DepthFrame& value,Clock::time_point received) {
                depth=value;last_time=value.captured_at;
                have_obs=true;obs_at=received;
            });
        } else {
            udp->Latest([&](const unsigned char* packet,std::size_t size) {
                auto value=g1_parkour::Decode(packet,size,policy.CameraPolicy().signature);
                if(running && have_obs && value.time-last_time>.05)
                    throw std::runtime_error("Observation frames dropped; history timing invalid");
                static_cast<g1_parkour::Observation&>(obs)=value;last_time=value.time;
                have_obs=true;obs_at=now;
            });
        }
        bool ready=state_fresh && have_obs && now-obs_at<std::chrono::milliseconds(150);
        if(hardware) {
            const double age=std::chrono::duration<double>(now.time_since_epoch()).count()-depth.captured_at;
            ready=ready && torso_fresh && age>=-.05 && age<.15;
        }
        auto stale_detail=[&]() {
            const double capture_age=have_obs ?
                (hardware ? (std::chrono::duration<double>(now.time_since_epoch()).count()-depth.captured_at)*1000 :
                 std::chrono::duration<double,std::milli>(now-obs_at).count()) : -1;
            return " camera_capture_age_ms="+std::to_string(capture_age)+
                " camera_receive_age_ms="+std::to_string(have_obs ? std::chrono::duration<double,std::milli>(now-obs_at).count() : -1)+
                " lowstate_age_ms="+std::to_string(state_age_ms)+
                " torso_imu_age_ms="+std::to_string(torso_age_ms);
        };
        if(!ready && (active||powered)) throw std::runtime_error("Camera observations or DDS state stale; stopping;"+stale_detail());
        if(!ready && now-waiting>g1_control::kStartupTimeout) throw std::runtime_error("No fresh camera/LowState/torso IMU;"+stale_detail());
        if(!ready) return false;
        waiting=now;
        if(hardware) {
            if(powered) limits->CheckState(snapshot,1u<<3);
            else g1_hardware::CheckTelemetryState(snapshot);
            static_cast<g1_parkour::Observation&>(obs)=g1_parkour::HardwareObservation(policy.CameraPolicy(),snapshot,imu,depth);
            remote.Update(snapshot.wireless_remote());
        } else {
            if(snapshot.mode_machine()!=0&&snapshot.mode_machine()!=5) throw std::runtime_error("Expected G1 29DOF");
            for(int i=0;i<29;++i) if(!std::isfinite(snapshot.motor_state()[i].q())||!std::isfinite(snapshot.motor_state()[i].dq()))
                throw std::runtime_error("Non-finite simulator state");
        }
        obs.lowstate = snapshot;
        if(powered && snapshot.mode_machine()!=command.mode_machine()) throw std::runtime_error("Robot mode changed");
        return true;
    }

    char ReadInput() {
        char key=keyboard->Read(!hardware&&!gamepad);
        if(gamepad) gamepad->Poll();
        if(input) key=g1_control::SelectAction(key,input->Action());
        return key;
    }

    void HandleInput(char key, Clock::time_point now) {
        if (key == '9' && !powered) TakeOver();
        if (key == 'p' || key == 'P') BeginReset(now);
        if ((key == 'r' || key == 'R') && holding && !running) TryStartPolicy();
    }

    void TakeOver() {
        if(!g1_control::InputReady(input))
            log.Line()<<"Center sticks before takeover\n";
        else {
            if(hardware) {
                limits->CheckState(snapshot,1u<<3);
                publisher=std::make_unique<g1_control::MotorPublisher>("rt/lowcmd");
                publisher->InitChannel();
                g1_control::ReleaseMotionService();
                powered = true;
                command.mode_machine() = snapshot.mode_machine();
                for(auto& motor:command.motor_cmd()) motor={};
                g1_control::SetDampingTargets(command, true, limits->stop_kd);
                Send();
                monitor->Arm(*publisher);
                // If any subsequent handover check fails, the exit path sends damping.
                if(!states.ReadFresh(snapshot,std::chrono::duration<double,std::milli>(limits->timeout_ms)))
                    throw std::runtime_error("State stale after service release");
                limits->CheckState(snapshot,1u<<3);
            } else {
                publisher=std::make_unique<g1_control::MotorPublisher>("rt/lowcmd");
                publisher->InitChannel();
                powered = true;
                command.mode_machine() = snapshot.mode_machine();
            }
            active = holding = running = false;
            velocity.fill(0);
            policy_worker->Pause();
            log.Line()<<"Output enabled; preview cleared. Press p / remote X to reset again.\n";
        }
    }

    void BeginReset(Clock::time_point now) {
        start=g1_control::JointPositions(snapshot);
        const auto defaults = policy.DefaultTargets();
        motor_kp = defaults.kp;
        motor_kd = defaults.kd;
        reset_at = now;
        active = true;
        holding = running = false;
        velocity.fill(0);
        policy_worker->Pause();
        command.mode_machine()=snapshot.mode_machine();
        log.Line()<<"Reset started ("<<(powered?"output":"dry-run")<<").\n";
    }

    void TryStartPolicy() {
        const bool near=g1_control::NearPose(g1_control::JointPositions(snapshot),neutral);
        if(!g1_control::InputReady(input))
            log.Line()<<"Center sticks before policy entry\n";
        else if(!powered||near) {
            policy_worker->Begin(powered);
            running = true;
            velocity.fill(0);
            if(input) input->BeginPolicy();
            log.Line()<<policy.Name()<<" policy ON."<<(powered?" Output active.\n":" Dry-run only.\n");
        } else log.Line()<<"Joints not near training pose; reset first\n";
    }

    void UpdatePolicy(char key) {
        if(running) {
            if(!hardware) switch(key) {
                case 'w':velocity[0]+=.1f;break;case 's':velocity[0]-=.1f;break;
                case 'q':velocity[2]+=.1f;break;case 'e':velocity[2]-=.1f;break;
                case 'x':velocity.fill(0);break;
            }
            if(input) velocity=input->ControlVelocity(hardware?0:key);
            policy.ClampVelocity(velocity);
            // 仿真允许低姿态和跌倒调试；此处只检查真机的躯干倾角。
            if(hardware && obs.gravity[2]>-std::cos(1.))
                throw std::runtime_error("Torso height/tilt protection triggered");
            policy_worker->Submit(obs,velocity);
            if(auto latest=policy_worker->Latest()) {
                target=latest->target.position;
                motor_kp=latest->target.kp;
                motor_kd=latest->target.kd;
            }
            // Until the first result, keep the reset/hold target. Latest()
            // enforces the same 60ms watchdog for startup and later stalls.
        }
    }

    void PublishTargets(Clock::time_point now) {
        if(active) {
            if(!running) {
                const double u=g1_control::ResetProgress(std::chrono::duration<double>(now-reset_at).count(),reset_seconds);
                target=g1_control::InterpolateResetPose(start,neutral,u);
                if(u==1&&!holding) {holding=true;log.Line()<<"Holding training pose; press r / remote A for policy.\n";}
            }
            g1_control::SetPositionTargets(command,target,motor_kp,motor_kd);
            if(powered) {
                if(hardware) limits->PrepareCommand(command);
                Send();telemetry.PublishTarget(command);
            } else {
                telemetry.PublishPreview(command);
            }
        } else if(powered&&hardware) {
            g1_control::SetDampingTargets(command, true, limits->stop_kd);
            Send();
        }
    }

    void Send() {
        if(!publisher) throw std::runtime_error("No authorized LowCmd publisher");
        command.crc()=g1_hardware::Crc(command);
        if(!publisher->Write(command)) throw std::runtime_error("LowCmd publication failed");
        publishing=true;
    }

    void ReleaseOutput() {
        if(publisher && (publishing||powered)) {
            g1_control::SetDampingTargets(command, hardware, hardware ? limits->stop_kd : 0);
            auto until=Clock::now()+std::chrono::duration<double>(hardware?limits->damping_seconds:.01);
            do {
                command.crc()=g1_hardware::Crc(command);
                if(!publisher->Write(command)) log.Line()<<"Stop DDS publication failed; execution unconfirmed\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } while(Clock::now()<until);
        }
        telemetry.PublishEnable(false);
    }

    void Stop() {
        // Invalidate in-flight work, damp outputs, then join the policy thread.
        policy_worker->Pause();
        ReleaseOutput();
        policy_worker.reset();
        keyboard.reset();
    }

    using PolicyWorker = g1_inference::PolicyWorker<ControlObservation, MotorTargets>;
    ControlPolicy& policy;
    const bool hardware;
    const std::string config, interface, joystick;
    const int domain, port;
    const double reset_seconds;
    std::unique_ptr<g1_hardware::Limits> limits;
    std::unique_ptr<g1_gamepad::Device> gamepad;
    g1_unitree_remote::Input remote;
    g1_gamepad::State* input = nullptr;
    std::unique_ptr<g1_parkour::UdpInput> udp;
    std::unique_ptr<g1_parkour::DepthReceiver> camera;
    g1_control::DdsStateCache<State> states;
    std::unique_ptr<g1_control::DdsStateCache<Imu>> imus;
    std::unique_ptr<g1_control::CommandMonitor> monitor;
    g1_control::MonitorPublisher telemetry;
    std::unique_ptr<g1_control::MotorPublisher> publisher;
    std::unique_ptr<g1_control::Keyboard> keyboard;
    g1_control::AsyncLog log;
    State snapshot{};
    Imu imu{};
    Cmd command{};
    bool powered=false, active=false, holding=false, running=false, publishing=false, have_obs=false;
    ControlObservation obs;
    g1_parkour::DepthFrame depth;
    g1_parkour::Joints start{}, target{}, neutral{}, motor_kp{}, motor_kd{};
    std::array<float, 3> velocity{};
    Clock::time_point obs_at{}, reset_at{}, waiting{};
    double last_time = -1;
    // Declared last: destruction joins inference before its buffers/dependencies disappear.
    std::unique_ptr<PolicyWorker> policy_worker;
};

}  // namespace

int RunControlLoop(const Options& options, ControlPolicy& policy, double reset_seconds) {
    return ParkourController(options, policy, reset_seconds).Run();
}

int RunController(const Options& options) {
    g1_control::InstallSignalHandlers();
    g1_parkour::Policy policy(options.config);
    if (options.mode == Mode::CheckPolicy) return CheckPolicy(policy);
    const auto reset_seconds = g1_config::ReadResetSeconds(YAML::LoadFile(options.config));
    ParkourPolicy adapter(policy);
    return RunControlLoop(options, adapter, reset_seconds);
}
}  // namespace g1_parkour_app
