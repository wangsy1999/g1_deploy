#include "g1_gamepad.h"
#include <iostream>

void Check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
bool Near(float a,float b) {return std::abs(a-b)<1e-5f;}
int main() {
    try {
        using g1_gamepad::State;
        // Match Linux joydev: mapping ioctls succeed with positive byte counts.
        auto query=[](unsigned long request,void* out) -> int {
            if(request==JSIOCGNAME(128)) {std::strcpy(static_cast<char*>(out),"Logitech Gamepad F710");return 22;}
            if(request==JSIOCGAXES) {*static_cast<unsigned char*>(out)=8;return 0;}
            if(request==JSIOCGBUTTONS) {*static_cast<unsigned char*>(out)=11;return 0;}
            if(request==JSIOCGAXMAP) {
                const std::array<unsigned char,8> axes{ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ,ABS_HAT0X,ABS_HAT0Y};
                std::copy(axes.begin(),axes.end(),static_cast<unsigned char*>(out));return ABS_CNT;
            }
            if(request==JSIOCGBTNMAP) {
                const std::array<unsigned short,11> buttons{BTN_A,BTN_B,BTN_X,BTN_Y,BTN_TL,BTN_TR,BTN_SELECT,BTN_START,BTN_MODE,BTN_THUMBL,BTN_THUMBR};
                std::copy(buttons.begin(),buttons.end(),static_cast<unsigned short*>(out));return 2*(KEY_MAX-BTN_MISC+1);
            }
            errno=EINVAL;return -1;
        };
        auto layout=g1_gamepad::ReadLayout(query);layout.Validate();
        Check(layout.axes==8 && layout.buttons==11 && layout.name=="Logitech Gamepad F710", "Positive ioctl byte counts must succeed");
        bool io_failed=false;
        try {g1_gamepad::ReadLayout([&](unsigned long request,void* out) {
            if(request==JSIOCGAXMAP) {errno=EIO;return -1;}return query(request,out);
        });} catch(const std::runtime_error& error) {io_failed=std::string(error.what()).find("JSIOCGAXMAP failed")!=std::string::npos;}
        Check(io_failed,"Failed ioctl must identify its operation");
        layout.axis_map[3]=ABS_RZ;
        bool bad_layout=false;
        try {layout.Validate();} catch(const std::runtime_error& error) {
            std::string message=error.what();bad_layout=message.find("ABS_RX")!=std::string::npos && message.find("Logitech Gamepad F710")!=std::string::npos;
        }
        Check(bad_layout,"Unsupported layout must report the missing axis and device");
        State locomotion(true);
        locomotion.Axis(ABS_Y,32767);locomotion.Axis(ABS_RX,-32767);
        Check(!locomotion.Ready(),"DreamWaQ requires lateral axis initialization");
        locomotion.Axis(ABS_X,-32767);locomotion.Button(BTN_TR,true);
        auto signed_velocity=locomotion.Velocity();
        Check(signed_velocity==std::array<float,3>{-1,1,1},"DreamWaQ signed reverse/lateral/yaw");
        locomotion.Axis(ABS_Y,0);locomotion.Axis(ABS_RX,0);
        Check(!locomotion.Centered(),"Lateral stick must be centered before policy entry");
        locomotion.Axis(ABS_X,0);Check(locomotion.Centered(),"Three-axis centering");
        locomotion.Axis(ABS_Y,-32767);locomotion.Axis(ABS_X,32767);locomotion.Axis(ABS_RX,32767);
        Check(locomotion.Velocity()==std::array<float,3>{1,-1,-1},"DreamWaQ forward/right/yaw");
        locomotion.Button(BTN_TR,false);
        Check(locomotion.Velocity()==std::array<float,3>{0,0,0},"DreamWaQ RB release zeros all axes");
        State s;
        Check(!s.Ready(),"Must receive both stick axes");
        s.Axis(ABS_Y,0);s.Axis(ABS_HAT0Y,-32767);
        Check(s.Sticks()[0]==0 && s.RawAxis(ABS_HAT0Y)==-32767,"D-pad channel must be visible without being treated as the left stick");
        s.Axis(ABS_Y,-32768);s.Axis(ABS_RX,-32768);
        Check(s.Ready(),"Both axes initialized");
        Check(s.Velocity()==std::array<float,3>{0,0,0},"RB must be held to move");
        s.Button(BTN_TR,true);
        auto v=s.Velocity();Check(Near(v[0],1)&&Near(v[2],1)&&v[1]==0,"Forward/left steering signs");
        s.Axis(ABS_Y,32767);s.Axis(ABS_RX,32767);v=s.Velocity();
        Check(v[0]==0 && Near(v[2],-1),"Reverse excluded; right steering negative");
        s.Axis(ABS_Y,1500);s.Axis(ABS_RX,-1500);
        Check(s.Centered() && s.Velocity()==std::array<float,3>{0,0,0},"Stick deadzone");
        Check(Near(State::Deadzone(.56f),.5f),"Deadzone rescaling");
        s.Button(BTN_TR,false);Check(!s.MovingEnabled(),"RB release");
        s.BeginPolicy();s.Button(BTN_TR,true);s.Axis(ABS_Y,-32767);
        Check(Near(s.ControlVelocity()[0],1),"Policy stick motion");
        Check(s.ControlVelocity('x')[0]==0 && s.ControlVelocity()[0]==0,"X must latch zero while RB is held");
        s.Button(BTN_TR,false);
        Check(s.ControlVelocity()[0]==0,"Release without centering does not rearm");
        s.Button(BTN_TR,true);Check(s.ControlVelocity()[0]==0,"Cannot bypass X latch by pressing RB again");
        s.Button(BTN_TR,false);s.Axis(ABS_Y,0);s.Axis(ABS_RX,0);s.ControlVelocity();
        s.Button(BTN_TR,true);s.Axis(ABS_Y,-32767);
        Check(Near(s.ControlVelocity()[0],1),"Center and release RB rearms motion");
        s.Button(BTN_START,true,true);Check(s.Action()==0,"Startup-held Start must not enable");
        s.Button(BTN_START,true);Check(s.Action()==0,"Held Start is not a new press");
        s.Button(BTN_START,false);s.Button(BTN_START,true);Check(s.Action()=='9',"Start enables");
        s.Button(BTN_START,true);Check(s.Action()==0,"No button auto-repeat");
        s.Button(BTN_A,true);Check(s.Action()=='p',"A resets");
        s.Button(BTN_Y,true);Check(s.Action()=='r',"Y enters policy");
        s.Button(BTN_X,true);Check(s.Action()=='x',"X zeros command");
        s.Button(BTN_START,false);s.Button(BTN_START,true);s.Button(BTN_B,true);
        Check(s.Action()==' ' && s.Action()==0,"B exit takes priority over pending actions");
        bool rejected=false;try {g1_gamepad::Device device("/path/to/nonexistent/f710");} catch(const std::runtime_error&) {rejected=true;}
        Check(rejected,"Missing joystick must fail without starting control");
        std::cout<<"Gamepad mapping, deadzone, RB gating, startup and edge tests passed\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
