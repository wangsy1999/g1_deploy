// Shared parkour networks; MuJoCo synchronized observations or real DDS + RealSense.
#include "g1_parkour_input.h"
#include "g1_policy_worker.h"
#include "g1_hardware.h"
#include "g1_motor_channel.h"
#include "g1_async_log.h"
#include "g1_unitree_remote.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <net/if.h>

using Clock=std::chrono::steady_clock;
using Cmd=unitree_hg::msg::dds_::LowCmd_;
using State=unitree_hg::msg::dds_::LowState_;
using Imu=unitree_hg::msg::dds_::IMUState_;
volatile std::sig_atomic_t stopped=0;
void Stop(int) {stopped=1;}

class Keyboard {
    termios saved_{}; bool restore_=false;
public:
    Keyboard() {
        if(!isatty(0)) return;
        if(tcgetattr(0,&saved_)) throw std::runtime_error("Cannot read terminal settings");
        auto s=saved_; s.c_lflag&=~(ICANON|ECHO); s.c_cc[VMIN]=0; s.c_cc[VTIME]=0;
        if(tcsetattr(0,TCSANOW,&s)) throw std::runtime_error("Cannot set terminal settings");
        restore_=true;
    }
    ~Keyboard() {if(restore_) tcsetattr(0,TCSANOW,&saved_);}
    char Read(bool eof_exits) {
        pollfd input{0,POLLIN,0}; int ready=poll(&input,1,0);
        if(ready<0 && errno!=EINTR) throw std::runtime_error("Keyboard poll failed");
        if(ready<=0) return 0;
        if(input.revents&(POLLERR|POLLNVAL)) throw std::runtime_error("Keyboard input failed");
        char key=0; auto n=read(0,&key,1);
        if(n<0 && errno!=EINTR && errno!=EAGAIN) throw std::runtime_error("Keyboard read failed");
        return n>0 ? key : n==0 && eof_exits ? ' ' : 0;
    }
};

void ReleaseService() {
    unitree::robot::b2::MotionSwitcherClient client;
    client.SetTimeout(5.0f); client.Init();
    for(int attempt=0;attempt<5 && !stopped;++attempt) {
        std::string form,name;
        if(client.CheckMode(form,name)) throw std::runtime_error("MotionSwitcher CheckMode failed");
        if(name.empty()) return;
        if(client.ReleaseMode()) throw std::runtime_error("MotionSwitcher ReleaseMode failed");
    }
    throw std::runtime_error("Motion service release not confirmed");
}

int main(int argc,char** argv) {
    std::signal(SIGINT,Stop); std::signal(SIGTERM,Stop); std::signal(SIGHUP,Stop);
    std::string models=G1_PARKOUR_DEFAULT_MODELS,config=G1_PARKOUR_DEFAULT_CONFIG;
    std::string limits_config=G1_POLICY_DEFAULT_CONFIG,interface="lo",joystick;
    bool hardware=false,check=false,calibrated=false,mode_set=false;
    int domain=1,port=0; double reset_seconds=3;
    try {
        for(int i=1;i<argc;++i) {
            std::string arg=argv[i];
            auto value=[&]() {if(i+1==argc) throw std::runtime_error("Missing value for "+arg);return std::string(argv[++i]);};
            if(arg=="--hardware" || arg=="--sim") {
                bool value=arg=="--hardware";
                if(mode_set && value!=hardware) throw std::runtime_error("--hardware and --sim cannot be combined");
                hardware=value;mode_set=true;
            }
            else if(arg=="--model-dir") models=value();
            else if(arg=="--config") config=value();
            else if(arg=="--limits") limits_config=value();
            else if(arg=="--interface") interface=value();
            else if(arg=="--domain") domain=std::stoi(value());
            else if(arg=="--port") port=std::stoi(value());
            else if(arg=="--reset-seconds") reset_seconds=std::stod(value());
            else if(arg=="--joystick") joystick=value();
            else if(arg=="--hardware-calibrated") calibrated=true;
            else if(arg=="--check-policy") check=true;
            else if(arg=="--help") {
                std::cout<<"g1_parkour --sim [--domain 1 --interface lo --port 17200]\n"
                    <<"g1_parkour --hardware --domain 0 --interface ROBOT_IF [--port 17201] [--hardware-calibrated]\n"
                    <<"Common: --model-dir DIR --config JSON --limits YAML --reset-seconds 3 --check-policy\n"
                    <<"Simulation F710: --joystick DEVICE. Hardware: Unitree Start enable, X reset, A policy, Select stop; direct sticks.\n"
                    <<"Startup is dry-run. p -> wait -> r previews targets; 9/Start hands over, clears preview, then p -> r again.\n"
                    <<"Hardware output needs --hardware-calibrated after verifying camera pose, torso IMU axes and joint signs.\n";
                return 0;
            } else throw std::runtime_error("Unknown argument: "+arg);
        }
        if(!std::isfinite(reset_seconds)||reset_seconds<=0 || (hardware ? domain!=0 : domain<=0))
            throw std::runtime_error("Invalid reset duration or DDS domain (hardware 0; sim positive)");
        if(!port) port=hardware?17201:17200;
        if(port<1024||port>65535) throw std::runtime_error("UDP port must be 1024..65535");
        if(hardware && (interface=="lo" || !if_nametoindex(interface.c_str()) || !joystick.empty()))
            throw std::runtime_error("Hardware requires a non-loopback robot interface and the Unitree remote");
        g1_parkour::Policy policy(models,config);
        if(check) {
            g1_parkour::Observation obs;obs.q=policy.neutral;obs.gravity={0,0,-1};obs.depth.fill(2.5f);
            auto target=policy.Step(obs,{0,0,0});
            std::cout<<"CPU parkour double-ONNX inference OK; DDS targets:";
            for(float v:target) std::cout<<' '<<v;
            std::cout<<'\n'; return 0;
        }
        std::unique_ptr<g1_hardware::Limits> limits;
        if(hardware) {
            limits=std::make_unique<g1_hardware::Limits>(YAML::LoadFile(limits_config));
            const auto pose=policy.DefaultDds();
            for(int i=0;i<29;++i) if(pose[i]<limits->lower[i]+.01f||pose[i]>limits->upper[i]-.01f)
                throw std::runtime_error("Parkour reset pose exceeds hardware limits");
        }
        std::unique_ptr<g1_gamepad::Device> gamepad;
        if(!joystick.empty()) gamepad=std::make_unique<g1_gamepad::Device>(joystick);
        g1_unitree_remote::Input remote;
        auto* input=hardware?&remote.state:gamepad?&gamepad->state:nullptr;
        std::unique_ptr<g1_parkour::UdpInput> udp;
        std::unique_ptr<g1_parkour::DepthReceiver> camera;
        if(hardware) camera=std::make_unique<g1_parkour::DepthReceiver>(port);
        else udp=std::make_unique<g1_parkour::UdpInput>(port);
        unitree::robot::ChannelFactory::Instance()->Init(domain,interface);
        // All captured storage outlives the subscribers. Receive callbacks only cache.
        std::mutex mutex; State state{}; Imu torso{};
        Clock::time_point state_at{},torso_at{}; bool have_state=false,have_torso=false;
        unitree::robot::ChannelSubscriber<State> states("rt/lowstate");
        states.InitChannel([&](const void* msg) {
            std::lock_guard<std::mutex> lock(mutex); state=*static_cast<const State*>(msg);
            state_at=Clock::now();have_state=true;
        },1);
        std::unique_ptr<unitree::robot::ChannelSubscriber<Imu>> imus;
        std::unique_ptr<g1_control::CommandMonitor> monitor;
        if(hardware) {
            imus=std::make_unique<unitree::robot::ChannelSubscriber<Imu>>("rt/secondary_imu");
            imus->InitChannel([&](const void* msg) {
                std::lock_guard<std::mutex> lock(mutex);torso=*static_cast<const Imu*>(msg);
                torso_at=Clock::now();have_torso=true;
            },1);
            monitor=std::make_unique<g1_control::CommandMonitor>();
        }
        unitree::robot::ChannelPublisher<Cmd> targets("rt/g1/reset_target");targets.InitChannel();
        unitree::robot::ChannelPublisher<Cmd> previews("rt/g1/dry_run_target");previews.InitChannel();
        unitree::robot::ChannelPublisher<Cmd> enables("rt/g1/reset_enable");enables.InitChannel();
        // No LowCmd publisher exists until explicit 9/Start takeover.
        std::unique_ptr<g1_control::MotorPublisher> publisher;
        Keyboard keyboard; Cmd command{};command.mode_pr()=0;
        bool powered=false,active=false,holding=false,running=false,publishing=false,have_obs=false;
        g1_parkour::Observation obs;g1_parkour::DepthFrame depth;
        g1_parkour::Joints start{},target{},previous{},neutral=policy.DefaultDds();
        std::array<float,3> velocity{};
        Clock::time_point obs_at{},reset_at{},next_log=Clock::now();
        g1_parkour::PolicyWorker<g1_parkour::Observation,g1_parkour::Joints> policy_worker(
            [&](const auto& observation,const auto& velocity) {return policy.Step(observation,velocity);},
            [&] {policy.Reset();}, !hardware);
        auto next=Clock::now(),waiting=next;
        double last_time=-1;
        auto enable_status=[&](bool on) {
            Cmd msg{};for(int i=0;i<29;++i) msg.motor_cmd()[i].mode()=on?1:0;
            msg.crc()=g1_hardware::Crc(msg);enables.Write(msg);
        };
        auto send=[&]() {
            if(!publisher) throw std::runtime_error("No authorized LowCmd publisher");
            command.crc()=g1_hardware::Crc(command);
            if(!publisher->Write(command)) throw std::runtime_error("LowCmd publication failed");
            publishing=true;
        };
        g1_control::AsyncLog log;
        auto release=[&]() {
            if(publisher && (publishing||powered)) {
                for(int i=0;i<29;++i) {
                    auto& motor=command.motor_cmd()[i];
                    motor.mode()=hardware?1:0;motor.kp()=motor.q()=motor.dq()=motor.tau()=0;
                    motor.kd()=hardware?limits->stop_kd:0;
                }
                auto until=Clock::now()+std::chrono::duration<double>(hardware?limits->damping_seconds:.01);
                do {
                    command.crc()=g1_hardware::Crc(command);
                    if(!publisher->Write(command)) log.Line()<<"Stop DDS publication failed; execution unconfirmed\n";
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } while(Clock::now()<until);
            }
            enable_status(false);
        };
        log.Line()<<"Parkour source="<<(hardware?"RealSense + torso DDS IMU":"MuJoCo synchronized camera observations")
                 <<"; loopback UDP "<<port<<"; DDS domain "<<domain<<".\n"
                 <<"DRY RUN: p reset, r policy; 9/Start enables real output and clears preview. Space/Select stops.\n";
        int result=0;
        try {
            while(!stopped) {
                auto now=Clock::now();
                if(powered && monitor && monitor->Conflict())
                    throw std::runtime_error("Competing LowCmd writer published after handover");
                State snapshot;Imu imu;bool state_fresh=false,torso_fresh=false;
                double state_age_ms=-1,torso_age_ms=-1;
                {
                    std::lock_guard<std::mutex> lock(mutex);snapshot=state;imu=torso;
                    state_fresh=have_state && now-state_at<std::chrono::milliseconds(hardware?int(limits->timeout_ms):500);
                    torso_fresh=have_torso && now-torso_at<std::chrono::milliseconds(100);
                    if(have_state) state_age_ms=std::chrono::duration<double,std::milli>(now-state_at).count();
                    if(have_torso) torso_age_ms=std::chrono::duration<double,std::milli>(now-torso_at).count();
                }
                if(hardware) {
                    camera->Latest([&](const g1_parkour::DepthFrame& value,Clock::time_point received) {
                        depth=value;last_time=value.captured_at;
                        have_obs=true;obs_at=received;
                    });
                } else {
                    udp->Latest([&](const unsigned char* packet,std::size_t size) {
                        auto value=g1_parkour::Decode(packet,size,policy.signature);
                        if(running && have_obs && value.time-last_time>.05)
                            throw std::runtime_error("Observation frames dropped; history timing invalid");
                        obs=value;last_time=value.time;
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
                if(!ready && now-waiting>std::chrono::seconds(10)) throw std::runtime_error("No fresh camera/LowState/torso IMU;"+stale_detail());
                if(!ready) {std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}
                waiting=now;
                if(hardware) {
                    if(powered) limits->CheckState(snapshot,1u<<3);
                    else g1_hardware::CheckTelemetryState(snapshot);
                    obs=g1_parkour::HardwareObservation(policy,snapshot,imu,depth);
                    remote.Update(snapshot.wireless_remote());
                } else {
                    if(snapshot.mode_machine()!=0&&snapshot.mode_machine()!=5) throw std::runtime_error("Expected G1 29DOF");
                    for(int i=0;i<29;++i) if(!std::isfinite(snapshot.motor_state()[i].q())||!std::isfinite(snapshot.motor_state()[i].dq()))
                        throw std::runtime_error("Non-finite simulator state");
                }
                if(powered && snapshot.mode_machine()!=command.mode_machine()) throw std::runtime_error("Robot mode changed");
                char key=keyboard.Read(!hardware&&!gamepad);
                if(gamepad) gamepad->Poll();
                if(input) {char action=input->Action();if(action && key!=' ') key=action;}
                if(key==' ') break;
                if(key=='9' && !powered) {
                    if(input && (!input->Ready()||!input->Centered()||(!hardware&&input->MovingEnabled())))
                        log.Line()<<"Center sticks before takeover\n";
                    else {
                        if(hardware) {
                            if(!calibrated) throw std::runtime_error("Hardware output requires --hardware-calibrated; verify camera mount, IMU axes and joint signs first");
                            limits->CheckState(snapshot,1u<<3);
                            publisher=std::make_unique<g1_control::MotorPublisher>("rt/lowcmd");
                            publisher->InitChannel();
                            ReleaseService();
                            powered=true;command.mode_machine()=snapshot.mode_machine();
                            for(auto& motor:command.motor_cmd()) motor={};
                            for(int i=0;i<29;++i) {
                                command.motor_cmd()[i].mode()=1;
                                command.motor_cmd()[i].kd()=limits->stop_kd;
                            }
                            send();
                            monitor->Arm(*publisher);
                            // If any subsequent handover check fails, the exit path sends damping.
                            {
                                std::lock_guard<std::mutex> lock(mutex);
                                if(!have_state || Clock::now()-state_at>std::chrono::duration<double,std::milli>(limits->timeout_ms))
                                    throw std::runtime_error("State stale after service release");
                                snapshot=state;
                            }
                            limits->CheckState(snapshot,1u<<3);
                        } else {
                            publisher=std::make_unique<g1_control::MotorPublisher>("rt/lowcmd");
                            publisher->InitChannel();powered=true;command.mode_machine()=snapshot.mode_machine();
                        }
                        for(int i=0;i<29;++i) previous[i]=snapshot.motor_state()[i].q();
                        active=holding=running=false;velocity.fill(0);policy_worker.Pause();
                        log.Line()<<"Output enabled; preview cleared. Press p / remote X to reset again.\n";
                    }
                }
                if(key=='p'||key=='P') {
                    for(int i=0;i<29;++i) start[i]=snapshot.motor_state()[i].q();
                    reset_at=now;active=true;holding=running=false;velocity.fill(0);policy_worker.Pause();
                    command.mode_machine()=snapshot.mode_machine();
                    log.Line()<<"Reset started ("<<(powered?"output":"dry-run")<<").\n";
                }
                if((key=='r'||key=='R') && holding && !running) {
                    bool near=true;for(int i=0;i<29;++i) if(std::abs(obs.q[i]-policy.neutral[i])>.35) near=false;
                    if(input && (!input->Ready()||!input->Centered()||(!hardware&&input->MovingEnabled())))
                        log.Line()<<"Center sticks before policy entry\n";
                    else if(!powered||near) {
                        policy_worker.Begin(powered);running=true;velocity.fill(0);
                        if(input) input->BeginPolicy();
                        log.Line()<<"Parkour policy ON."<<(powered?" Output active.\n":" Dry-run only.\n");
                    } else log.Line()<<"Joints not near training pose; reset first\n";
                }
                double policy_target_age_ms=-1;
                if(running) {
                    if(!hardware) switch(key) {
                        case 'w':velocity[0]+=.1f;break;case 's':velocity[0]-=.1f;break;
                        case 'q':velocity[2]+=.1f;break;case 'e':velocity[2]-=.1f;break;
                        case 'x':velocity.fill(0);break;
                    }
                    if(input) velocity=input->ControlVelocity(hardware?0:key);
                    velocity[0]=std::clamp(velocity[0],0.f,1.f);velocity[1]=0;
                    velocity[2]=std::clamp(velocity[2],-1.f,1.f);
                    if((!hardware&&obs.height<.5)||obs.gravity[2]>-std::cos(1.))
                        throw std::runtime_error("Torso height/tilt protection triggered");
                    policy_worker.Submit(obs,velocity);
                    if(auto latest=policy_worker.Latest()) {
                        target=latest->target;
                        policy_target_age_ms=std::chrono::duration<double,std::milli>(Clock::now()-latest->completed_at).count();
                    }
                    // Until the first result, keep the reset/hold target. Latest()
                    // enforces the same 60ms watchdog for startup and later stalls.
                }

                if(active) {
                    if(!running) {
                        double u=std::clamp(std::chrono::duration<double>(now-reset_at).count()/reset_seconds,0.,1.);
                        double blend=u*u*(3-2*u);
                        for(int i=0;i<29;++i) target[i]=start[i]+blend*(neutral[i]-start[i]);
                        if(u==1&&!holding) {holding=true;log.Line()<<"Holding training pose; press r / remote A for parkour.\n";}
                    }
                    for(int i=0;i<29;++i) {
                        auto& motor=command.motor_cmd()[policy.ids[i]];motor.mode()=1;
                        motor.q()=target[policy.ids[i]];motor.kp()=policy.kp[i];motor.kd()=policy.kd[i];
                        motor.dq()=motor.tau()=0;
                    }
                    if(powered) {
                        if(hardware) limits->LimitCommand(command,previous,.002);
                        send();targets.Write(command);
                    } else {
                        auto preview=command;for(auto& motor:preview.motor_cmd()) motor.mode()=0;
                        preview.crc()=g1_hardware::Crc(preview);previews.Write(preview);
                    }
                } else if(powered&&hardware) {
                    for(int i=0;i<29;++i) {
                        auto& motor=command.motor_cmd()[i];
                        motor.mode()=1;motor.q()=motor.dq()=motor.kp()=motor.tau()=0;motor.kd()=limits->stop_kd;
                    }
                    send();
                }
                enable_status(powered);
                if(now>=next_log) {
                    auto entry=log.Line();
                    entry<<"[parkour] "<<(powered?"OUTPUT":"DRY")<<" "<<(running?"policy":holding?"hold":active?"reset":"waiting")
                        <<" camera_age_ms="<<(hardware ?
                           (std::chrono::duration<double>(now.time_since_epoch()).count()-depth.captured_at)*1000 :
                           std::chrono::duration<double,std::milli>(now-obs_at).count())
                        <<" policy_target_age_ms="<<policy_target_age_ms
                        <<" velocity=["<<velocity[0]<<",0,"<<velocity[2]<<"]";
                    if(input) entry<<" sticks=["<<input->Sticks()[0]<<','<<input->Sticks()[1]<<"]";
                    entry<<'\n';next_log=now+std::chrono::milliseconds(200);
                }
                const auto control_period=std::chrono::milliseconds(2);
                next+=control_period;
                const auto after=Clock::now();
                if(next<=after) next+=control_period*((after-next)/control_period+1);
                std::this_thread::sleep_until(next);
            }
        } catch(const std::exception& error) {log.Line()<<"Parkour stopped: "<<error.what()<<'\n';result=1;}
        // Invalidate in-flight inference before damping. Do not wait for the
        // inference thread to join until after release() has stopped output.
        policy_worker.Pause();
        release();
        return result;
    } catch(const std::exception& error) {std::cerr<<"g1_parkour: "<<error.what()<<'\n';return 1;}
}
