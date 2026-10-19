#include "g1_unitree_remote.h"
#include <iostream>
#include <limits>

void Check(bool yes,const char* why) {if(!yes) throw std::runtime_error(why);}
std::array<uint8_t,40> Packet(unsigned keys=0,float lx=0,float rx=0,float ly=0) {
    std::array<uint8_t,40> raw{};
    raw[0]=0x55;raw[1]=0x51;raw[2]=keys&255;raw[3]=(keys>>8)&255;
    const std::array<float,5> values{lx,rx,0,0,ly};
    for(int i=0;i<5;++i) {
        uint32_t bits;std::memcpy(&bits,&values[i],4);
        for(int j=0;j<4;++j) raw[4+4*i+j]=(bits>>(8*j))&255;
    }
    return raw;
}
int main() {
    try {
        g1_unitree_remote::Input remote;
        remote.Update(Packet(1u<<2));
        Check(remote.state.Ready() && remote.state.Centered(),"Remote axes initialize");
        Check(remote.state.Action()==0,"Startup-held Start must not enable output");
        remote.Update(Packet());remote.Update(Packet(1u<<2));
        Check(remote.state.Action()=='9',"Start maps to real handover");
        remote.Update(Packet(1u<<2));Check(remote.state.Action()==0,"No Start auto-repeat");
        remote.Update(Packet(1u<<10));Check(remote.state.Action()=='p',"X maps to reset");
        remote.Update(Packet(1u<<8));Check(remote.state.Action()=='r',"A maps to policy");
        remote.state.BeginPolicy();
        remote.Update(Packet(0,1,1,1));
        Check(remote.state.ControlVelocity()==std::array<float,3>{1,-1,-1},"Direct forward/right/yaw, without R1");
        remote.Update(Packet(0,-1,-1,-1));
        Check(remote.state.ControlVelocity()==std::array<float,3>{-1,1,1},"Reverse/left/yaw signs");
        remote.Update(Packet(1,1,1,1));
        Check(remote.state.ControlVelocity()==std::array<float,3>{1,-1,-1},"R1 does not alter direct control");
        remote.Update(Packet(0,.05f,-.05f,.05f));
        Check(remote.state.Centered() && remote.state.ControlVelocity()==std::array<float,3>{0,0,0},"Release/deadzone zeros velocity");
        remote.Update(Packet(0,0,0,1));
        Check(remote.state.ControlVelocity('x')==std::array<float,3>{0,0,0},"Keyboard X zero lock");
        Check(remote.state.ControlVelocity()==std::array<float,3>{0,0,0},"Zero stays locked until sticks center");
        remote.Update(Packet());remote.state.ControlVelocity();remote.Update(Packet(0,0,0,1));
        Check(remote.state.ControlVelocity()[0]==1,"Centering rearms direct sticks");
        remote.Update(Packet());remote.Update(Packet((1u<<2)|(1u<<3)|(1u<<8)|(1u<<10)));
        Check(remote.state.Action()==' ' && remote.state.Action()==0,"Select stop overrides pending Start/reset/policy");
        g1_unitree_remote::Input held_stop;held_stop.Update(Packet(1u<<3));
        Check(held_stop.state.Action()==' ',"Startup-held Select must stop");
        g1_unitree_remote::Input held_actions;held_actions.Update(Packet((1u<<8)|(1u<<10)));
        Check(held_actions.state.Action()==0,"Suppress startup-held reset/policy");
        for(float bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),1.2f}) {
            bool rejected=false;
            try {remote.Update(Packet(0,bad));}catch(const std::runtime_error&){rejected=true;}
            Check(rejected,"Reject malformed wireless axis");
        }
        std::cout<<"Unitree remote wire decode, direct sticks, deadzone, Start/X/A/Select, startup and stop priority passed\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
