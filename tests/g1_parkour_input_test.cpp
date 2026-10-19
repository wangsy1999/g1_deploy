#include "g1_parkour_input.h"
#include <iostream>
#include <limits>

void Check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
int main(int argc,char** argv) {
    try {
        Check(argc==3,"Expected models and config");
        g1_parkour::Policy policy(argv[1],argv[2]);
        std::array<unsigned char,32+36*64*4> bytes{};
        std::memcpy(bytes.data(),"G1DEP001",8);
        uint64_t seq=17;double time=12.5;uint32_t w=64,h=36;
        std::memcpy(bytes.data()+8,&seq,8);std::memcpy(bytes.data()+16,&time,8);
        std::memcpy(bytes.data()+24,&w,4);std::memcpy(bytes.data()+28,&h,4);
        std::array<float,36*64> image;image.fill(1.25f);
        std::memcpy(bytes.data()+32,image.data(),image.size()*4);
        auto depth=g1_parkour::DecodeDepth(bytes.data(),bytes.size());
        Check(depth.sequence==17 && depth.captured_at==12.5 && depth.image[0]==1.25f,"Python/C++ depth packet layout");
        bool rejected=false;
        try {g1_parkour::DecodeDepth(bytes.data(),bytes.size()-1);} catch(...) {rejected=true;}
        Check(rejected,"Reject truncated depth packet");
        float nan=std::numeric_limits<float>::quiet_NaN();std::memcpy(bytes.data()+32,&nan,4);
        rejected=false;try {g1_parkour::DecodeDepth(bytes.data(),bytes.size());} catch(...) {rejected=true;}
        Check(rejected,"Reject NaN depth");
        unitree_hg::msg::dds_::LowState_ state{};
        unitree_hg::msg::dds_::IMUState_ torso{};
        torso.quaternion()={1,0,0,0};torso.gyroscope()={1,2,3};
        // Pelvis has a different orientation/gyro: verify it never feeds the torso-root policy.
        state.imu_state().quaternion()={0,1,0,0};state.imu_state().gyroscope()={9,9,9};
        for(int i=0;i<29;++i) {state.motor_state()[i].q()=.01f*i;state.motor_state()[i].dq()=.1f*i;}
        auto obs=g1_parkour::HardwareObservation(policy,state,torso,depth);
        Check(obs.gravity==std::array<float,3>{0,0,-1} && obs.omega==std::array<float,3>{1,2,3},"Use torso IMU, not pelvis");
        for(int i=0;i<29;++i) {
            Check(obs.q[i]==policy.sign[i]*state.motor_state()[policy.ids[i]].q(),"Real q maps to training coordinates");
            Check(obs.dq[i]==policy.sign[i]*state.motor_state()[policy.ids[i]].dq(),"Real dq maps to training coordinates");
        }
        torso.quaternion()={0,0,0,0};rejected=false;
        try {g1_parkour::HardwareObservation(policy,state,torso,depth);} catch(...) {rejected=true;}
        Check(rejected,"Reject invalid torso orientation");
        std::cout<<"Depth protocol, real joint mapping and torso IMU checks passed\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
