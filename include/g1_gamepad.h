#pragma once
// Linux jsdev input. Look up evdev codes instead of assuming joystick axis indices.
#include <linux/input.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <string>
#include <sstream>
#include <utility>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace g1_gamepad {
class State {
    std::array<float,ABS_CNT> axes_{};
    std::array<short,ABS_CNT> raw_axes_{};
    std::array<bool,KEY_CNT> buttons_{};
    std::deque<char> actions_;
    bool have_y_=false, have_rx_=false;
    bool stop_=false;
    bool motion_armed_=false;
    bool locomotion_=false, have_x_=false;
public:
    explicit State(bool locomotion=false):locomotion_(locomotion) {}
    static float Deadzone(float v) {
        constexpr float dead=.12f;
        v=std::clamp(v,-1.f,1.f);
        return std::abs(v)<=dead ? 0.f : std::copysign((std::abs(v)-dead)/(1-dead),v);
    }
    void Axis(unsigned code,short value) {
        if(code>=axes_.size()) return;
        raw_axes_[code]=value;
        axes_[code]=std::clamp(float(value)/32767.f,-1.f,1.f);
        if(code==ABS_Y) have_y_=true;
        if(code==ABS_RX) have_rx_=true;
        if(code==ABS_X) have_x_=true;
    }
    void Button(unsigned code,bool down,bool initial=false) {
        if(code>=buttons_.size()) return;
        bool edge=down && !buttons_[code] && !initial;
        buttons_[code]=down;
        if(!edge) return;
        char action=0;
        switch(code) {
            case BTN_START:action='9';break;
            case BTN_A:action='p';break;
            case BTN_Y:action='r';break;
            case BTN_B:stop_=true;break;
            case BTN_X:action='x';break;
        }
        if(action) actions_.push_back(action);
    }
    char Action() {
        if(stop_) {stop_=false;actions_.clear();return ' ';}
        if(actions_.empty()) return 0;
        char result=actions_.front();actions_.pop_front();return result;
    }
    bool Ready() const {return have_y_ && have_rx_ && (!locomotion_ || have_x_);}
    short RawAxis(unsigned code) const {return code<raw_axes_.size()?raw_axes_[code]:0;}
    bool MovingEnabled() const {return buttons_[BTN_TR];}
    bool Centered() const {return Deadzone(axes_[ABS_Y])==0 && Deadzone(axes_[ABS_RX])==0 && (!locomotion_ || Deadzone(axes_[ABS_X])==0);}
    std::array<float,2> Sticks() const {return {-Deadzone(axes_[ABS_Y]),-Deadzone(axes_[ABS_RX])};}
    std::array<float,3> Velocity() const {
        if(!Ready() || !MovingEnabled()) return {0,0,0};
        if(locomotion_) return {-Deadzone(axes_[ABS_Y]),-Deadzone(axes_[ABS_X]),-Deadzone(axes_[ABS_RX])};
        return {std::clamp(-Deadzone(axes_[ABS_Y]),0.f,1.f),0,-Deadzone(axes_[ABS_RX])};
    }
    void BeginPolicy() {motion_armed_=true;}
    std::array<float,3> ControlVelocity(char action=0) {
        if(!MovingEnabled() && Centered()) motion_armed_=true;
        if(action=='x'||action=='X') motion_armed_=false;
        return motion_armed_ ? Velocity() : std::array<float,3>{0,0,0};
    }
};

// JSIOCGNAME, JSIOCGAXMAP and JSIOCGBTNMAP may return a positive byte count.
// Injecting the ioctl query allows regression checks without a physical USB device.
struct Layout {
    std::array<unsigned char,ABS_CNT> axis_map{};
    std::array<unsigned short,KEY_MAX-BTN_MISC+1> button_map{};
    unsigned char axes=0, buttons=0;
    std::string name;
    std::string Describe() const {
        std::ostringstream text;
        text<<"name='"<<name<<"', axes="<<int(axes)<<" [";
        for(unsigned i=0;i<std::min<std::size_t>(axes,axis_map.size());++i)
            text<<(i?", ":"")<<i<<":0x"<<std::hex<<int(axis_map[i])<<std::dec;
        text<<"], buttons="<<int(buttons)<<" [";
        for(unsigned i=0;i<std::min<std::size_t>(buttons,button_map.size());++i)
            text<<(i?", ":"")<<i<<":0x"<<std::hex<<button_map[i]<<std::dec;
        text<<"]";return text.str();
    }
    void Validate(bool locomotion=false) const {
        if(axes>axis_map.size() || buttons>button_map.size())
            throw std::runtime_error("Gamepad counts exceed mapping capacity: "+Describe());
        auto has_axis=[&](int code) {return std::find(axis_map.begin(),axis_map.begin()+axes,code)!=axis_map.begin()+axes;};
        auto has_button=[&](int code) {return std::find(button_map.begin(),button_map.begin()+buttons,code)!=button_map.begin()+buttons;};
        std::string missing;
        if(!has_axis(ABS_Y)) missing+=" ABS_Y";
        if(!has_axis(ABS_RX)) missing+=" ABS_RX";
        if(locomotion && !has_axis(ABS_X)) missing+=" ABS_X";
        for(const auto& item:std::array<std::pair<int,const char*>,6>{{
            {BTN_START,"Start"},{BTN_A,"A"},{BTN_B,"B"},{BTN_X,"X"},{BTN_Y,"Y"},{BTN_TR,"RB"}}})
            if(!has_button(item.first)) missing+=" "+std::string(item.second);
        if(!missing.empty()) throw std::runtime_error("Unsupported gamepad layout; missing:"+missing+
            ". "+Describe()+". Check F710 X/D mode and selected device.");
    }
};

template<class Query> Layout ReadLayout(Query query) {
    Layout layout;
    char name[128]{};
    auto read=[&](unsigned long request,void* output,const char* operation) {
        if(query(request,output)<0)
            throw std::runtime_error(std::string(operation)+" failed: "+std::strerror(errno));
    };
    read(JSIOCGNAME(sizeof(name)),name,"JSIOCGNAME");
    name[sizeof(name)-1]=0;layout.name=name;
    read(JSIOCGAXES,&layout.axes,"JSIOCGAXES");
    read(JSIOCGBUTTONS,&layout.buttons,"JSIOCGBUTTONS");
    read(JSIOCGAXMAP,layout.axis_map.data(),"JSIOCGAXMAP");
    read(JSIOCGBTNMAP,layout.button_map.data(),"JSIOCGBTNMAP");
    return layout;
}

class Device {
    int fd_=-1;
    Layout layout_;
public:
    State state;
    std::string name;
    std::string Description() const {return layout_.Describe();}
    std::string RawAxes() const {
        std::ostringstream text;
        text<<"raw_axes=[";
        for(unsigned i=0;i<layout_.axes;++i)
            text<<(i?", ":"")<<i<<"(0x"<<std::hex<<int(layout_.axis_map[i])<<std::dec<<")="<<state.RawAxis(layout_.axis_map[i]);
        text<<"]";return text.str();
    }
    explicit Device(const std::string& path,bool locomotion=false):state(locomotion) {
        fd_=open(path.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
        if(fd_<0) throw std::runtime_error("Cannot open gamepad "+path+": "+std::strerror(errno));
        try {
            layout_=ReadLayout([&](unsigned long request,void* output) {return ioctl(fd_,request,output);});
            layout_.Validate(locomotion);
            name=layout_.name;
            Poll();
        } catch(...) {close(fd_);fd_=-1;throw;}
    }
    ~Device() {if(fd_>=0) close(fd_);}
    Device(const Device&)=delete;
    Device& operator=(const Device&)=delete;
    void Poll() {
        pollfd descriptor{fd_,POLLIN,0};
        int ready=poll(&descriptor,1,0);
        if(ready<0 && errno!=EINTR) throw std::runtime_error("Gamepad poll failed");
        if(descriptor.revents&(POLLHUP|POLLERR|POLLNVAL)) throw std::runtime_error("Gamepad disconnected");
        for(int i=0;i<256;++i) {
            js_event event{};auto n=read(fd_,&event,sizeof(event));
            if(n<0) {
                if(errno==EAGAIN || errno==EWOULDBLOCK) break;
                if(errno==EINTR) continue;
                throw std::runtime_error("Gamepad read failed/disconnected");
            }
            if(n!=sizeof(event)) throw std::runtime_error("Gamepad closed/truncated event");
            const bool initial=event.type&JS_EVENT_INIT;
            if((event.type&~JS_EVENT_INIT)==JS_EVENT_AXIS && event.number<layout_.axes)
                state.Axis(layout_.axis_map[event.number],event.value);
            if((event.type&~JS_EVENT_INIT)==JS_EVENT_BUTTON && event.number<layout_.buttons)
                state.Button(layout_.button_map[event.number],event.value!=0,initial);
        }
    }
};

inline void CheckInput(Device& gamepad,const volatile std::sig_atomic_t& stopped) {
    std::cout<<"Input check only; no DDS or policy commands. Ctrl+C or B to exit.\n"
             <<"F710: MODE light should be OFF. Left stick uses ABS_X (0x0) / ABS_Y (0x1).\n"
             <<"If only ABS_HAT0Y (0x11) changes, MODE has swapped left stick and D-pad.\n"<<std::flush;
    while(!stopped) {
        gamepad.Poll();
        char action=gamepad.state.Action();
        auto sticks=gamepad.state.Sticks();auto velocity=gamepad.state.Velocity();
        float lateral=-State::Deadzone(float(gamepad.state.RawAxis(ABS_X))/32767.f);
        std::cout<<"forward="<<sticks[0]<<" lateral="<<lateral<<" yaw="<<sticks[1]
                 <<" RB="<<gamepad.state.MovingEnabled()<<" command(normalized)=["<<velocity[0]<<","<<velocity[1]<<","<<velocity[2]
                 <<"] action="<<(action==' '?'B':action? action:'-')<<" "<<gamepad.RawAxes()<<'\n'<<std::flush;
        if(action==' ') break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
} // namespace g1_gamepad
