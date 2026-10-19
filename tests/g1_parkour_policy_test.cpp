#include "g1_parkour_policy.h"
#include <iostream>
#include <limits>

void Check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
int main(int argc,char** argv) {
    try {
        Check(argc==3,"Expected models and config");
        g1_parkour::Policy policy(argv[1],argv[2]);
        g1_parkour::Observation obs;obs.q=policy.neutral;obs.gravity={0,0,-1};
        std::pair<std::vector<float>,std::vector<float>> inputs;
        for(int t=0;t<40;++t) {
            obs.omega.fill(t*.01f);obs.depth.fill(t*.01f);
            inputs=policy.Observe(obs,{0,0,0});
        }
        for(int i=0;i<8;++i) {
            Check(std::abs(inputs.first[i*3]-(32+i)*.01f*.25f)<1e-6,"Proprio history order");
            Check(std::abs(inputs.second[i*18*32]-(4+5*i)*.01f/2.5f)<1e-6,"Depth stride/order");
        }
        policy.Reset();obs.depth.fill(2.5f);obs.omega.fill(0);
        auto target=policy.Step(obs,{0,0,0});
        auto after=policy.Observe(obs,{0,0,0});
        for(int i=0;i<29;++i) {
            const auto raw=(policy.sign[i]*target[policy.ids[i]]-policy.neutral[i])/policy.scale[i];
            Check(std::abs(after.first[768-29+i]-raw)<1e-5,"Last action feedback");
        }
        obs.depth.fill(std::numeric_limits<float>::infinity());
        for(float v:policy.Preprocess(obs.depth)) Check(v==1,"Infinite no-hit preprocessing");
        obs.depth[0]=-1;bool rejected=false;
        try {policy.Preprocess(obs.depth);} catch(const std::runtime_error&) {rejected=true;}
        Check(rejected,"Negative depth must fail");
        std::array<unsigned char,9508> bytes{};std::memcpy(bytes.data(),"G1PKR001",8);
        uint32_t count=2369;std::memcpy(bytes.data()+24,&policy.signature,4);std::memcpy(bytes.data()+28,&count,4);
        auto decoded=g1_parkour::Decode(bytes.data(),bytes.size(),policy.signature);
        Check(decoded.sequence==0,"Packet decoding");
        rejected=false;try {g1_parkour::Decode(bytes.data(),bytes.size()-1,policy.signature);} catch(...) {rejected=true;}
        Check(rejected,"Truncated packet must fail");
        std::cout<<"Parkour history, depth, feedback and protocol checks passed\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
