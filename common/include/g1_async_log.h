#pragma once
#include <atomic>
#include <cerrno>
#include <chrono>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <iostream>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

namespace g1_control {
// Logging is best effort: bounded queue, try-lock producer, nonblocking sink.
// Opening /proc/self/fd creates an independent file description, so O_NONBLOCK
// does not change stdout flags for other users of the original descriptor.
class AsyncLog {
    std::mutex mutex_;
    std::deque<std::string> pending_;
    std::atomic<bool> closed_{false}, writing_{false};
    static int Open(int output) {
        // Flush startup text before opening an independent append descriptor.
        // This runs before the control loop, never on its logging path.
        if(output==STDOUT_FILENO) std::cout.flush();
        return open(("/proc/self/fd/"+std::to_string(output)).c_str(),O_WRONLY|O_NONBLOCK|O_APPEND|O_CLOEXEC);
    }
    int fd_;
    std::thread thread_;
    void Submit(std::string message) noexcept {
        try {
            std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
            if(lock && pending_.size()<64) pending_.push_back(std::move(message));
        } catch(...) {} // Logging failure must never interrupt motor publication.
    }
    void Run() {
        sigset_t signals;sigemptyset(&signals);sigaddset(&signals,SIGPIPE);
        pthread_sigmask(SIG_BLOCK,&signals,nullptr);
        while(!closed_) {
            std::string message;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!pending_.empty()) {writing_=true;message=std::move(pending_.front());pending_.pop_front();}
            }
            if(message.empty()) {writing_=false;std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}
            std::size_t offset=0;
            while(fd_>=0 && offset<message.size() && !closed_) {
                pollfd output{fd_,POLLOUT,0};
                if(poll(&output,1,10)<=0) continue;
                if(output.revents&(POLLERR|POLLHUP|POLLNVAL)) break;
                const auto count=write(fd_,message.data()+offset,message.size()-offset);
                if(count>0) offset+=count;
                else if(count<0 && (errno==EINTR || errno==EAGAIN)) continue;
                else break;
            }
            writing_=false;
        }
    }
public:
    class Entry: public std::ostringstream {
        AsyncLog& log_;
    public:
        explicit Entry(AsyncLog& log):log_(log) {}
        ~Entry() noexcept {try {log_.Submit(str());} catch(...) {}}
    };
    explicit AsyncLog(int output=STDOUT_FILENO)
        :fd_(Open(output)),
         thread_([this] {Run();}) {}
    ~AsyncLog() {
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(20);
        while(std::chrono::steady_clock::now()<deadline) {
            {std::lock_guard<std::mutex> lock(mutex_);if(pending_.empty() && !writing_) break;}
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        closed_=true;if(thread_.joinable()) thread_.join();if(fd_>=0) close(fd_);
    }
    Entry Line() {return Entry(*this);}
};
} // namespace g1_control
