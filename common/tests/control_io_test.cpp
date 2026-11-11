#include "g1_async_log.h"
#include "g1_motor_channel.h"
#include <iostream>
#include <stdexcept>

void Require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
int main() {
    try {
        // Saturate a pipe without a reader. Logging and teardown must still finish.
        int pipefd[2];Require(pipe(pipefd)==0,"pipe failed");
        int fill=open(("/proc/self/fd/"+std::to_string(pipefd[1])).c_str(),O_WRONLY|O_NONBLOCK);
        char bytes[4096]{};while(write(fill,bytes,sizeof(bytes))>0) {}
        const auto start=std::chrono::steady_clock::now();
        {
            g1_control::AsyncLog log(pipefd[1]);
            for(int i=0;i<1000;++i) log.Line()<<"backpressured log "<<i<<'\n';
            Require(!(fcntl(pipefd[1],F_GETFL)&O_NONBLOCK),"Logger changed original descriptor flags");
        }
        Require(std::chrono::steady_clock::now()-start<std::chrono::milliseconds(250),"Logging blocked on full pipe");
        close(fill);close(pipefd[0]);close(pipefd[1]);
        // A closed sink must not terminate the controller with SIGPIPE.
        Require(pipe(pipefd)==0,"pipe failed");
        {g1_control::AsyncLog log(pipefd[1]);close(pipefd[0]);log.Line()<<"closed pipe\n";}
        close(pipefd[1]);
        // Isolated positive DDS domain; no hardware handover or robot connection.
        unitree::robot::ChannelFactory::Instance()->Init(199,"lo");
        g1_control::CommandMonitor monitor;
        g1_control::MotorPublisher own("rt/lowcmd"),other("rt/lowcmd");
        own.InitChannel();other.InitChannel();
        g1_control::MotorCommand command;
        own.Write(command);
        monitor.Arm(own);
        for(int i=0;i<50;++i) {Require(own.Write(command),"Own write failed");std::this_thread::sleep_for(std::chrono::milliseconds(2));}
        Require(!monitor.Conflict(),"Own writer misidentified as competitor");
        for(int i=0;i<100 && !monitor.Conflict();++i) {other.Write(command);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
        Require(monitor.Conflict(),"Second writer not detected");
        std::cout<<"Nonblocking logs and DDS writer identity checks passed\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
