#include "g1_policy_worker.h"
#include <atomic>
#include <iostream>
#include <string>

using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
struct Observation {uint64_t sequence=0;};
using Worker=g1_parkour::PolicyWorker<Observation,int>;
void Require(bool condition,const char* message) {
    if(!condition) throw std::runtime_error(message);
}
template<class Predicate> void Wait(Predicate predicate) {
    const auto deadline=Clock::now()+500ms;
    while(!predicate()) {
        if(Clock::now()>=deadline) throw std::runtime_error("Test wait timed out");
        std::this_thread::sleep_for(1ms);
    }
}

int main() {
    try {
        // A deliberately blocked inference must not block the control mailbox.
        // A reset during that inference must discard its result and pending input.
        {
            std::mutex mutex;
            std::condition_variable changed;
            bool entered=false,release=false;
            std::atomic<int> resets{0};
            Worker worker([&](const Observation& obs,const auto&) {
                if(obs.sequence==1) {
                    std::unique_lock<std::mutex> lock(mutex);
                    entered=true;changed.notify_all();
                    if(!changed.wait_for(lock,500ms,[&] {return release;}))
                        throw std::runtime_error("Blocked inference not released");
                }
                return int(obs.sequence);
            },[&] {++resets;});
            worker.Begin(false);worker.Submit({1},{});
            {
                std::unique_lock<std::mutex> lock(mutex);
                Require(changed.wait_for(lock,500ms,[&] {return entered;}),"Inference did not start");
            }
            for(int tick=0;tick<5;++tick) {
                worker.Submit({2},{});
                Require(!worker.Latest(),"Unexpected result while inference blocked");
            }
            worker.Pause();worker.Begin(false);worker.Submit({3},{});
            {std::lock_guard<std::mutex> lock(mutex);release=true;}
            changed.notify_all();
            Wait([&] {auto result=worker.Latest();return result && result->target==3;});
            Require(resets==2,"Policy state was not reset for the new generation");
            worker.Pause();
            Require(!worker.Latest(),"Paused target was retained");
        }
        // Repeated simulator snapshots must not append duplicate history frames.
        {
            std::atomic<int> calls{0};
            Worker worker([&](const auto& obs,const auto&) {++calls;return int(obs.sequence);},[] {},true);
            worker.Begin(false);worker.Submit({1},{});
            Wait([&] {return bool(worker.Latest());});
            worker.Submit({1},{});
            std::this_thread::sleep_for(25ms);
            Require(calls==1,"Duplicate simulator frame inferred twice");
            worker.Submit({2},{});
            Wait([&] {auto result=worker.Latest();return result && result->target==2;});
        }
        // Exceptions cross the mailbox; a new generation can reset after failure.
        {
            Worker worker([](const auto& obs,const auto&) {
                if(obs.sequence==1) throw std::runtime_error("inference failure");
                return int(obs.sequence);
            },[] {});
            worker.Begin(false);worker.Submit({1},{});
            Wait([&] {
                try {worker.Latest();return false;}
                catch(const std::runtime_error& error) {return std::string(error.what())=="inference failure";}
            });
            worker.Begin(false);worker.Submit({2},{});
            Wait([&] {auto result=worker.Latest();return result && result->target==2;});
        }
        // Both first-result timeout and stale established targets stop the caller.
        {
            Worker worker([](const auto&,const auto&) {return 1;},[] {});
            worker.Begin(false);
            std::this_thread::sleep_for(Worker::target_timeout+5ms);
            bool expired=false;
            try {worker.Latest();} catch(const std::runtime_error&) {expired=true;}
            Require(expired,"First-result watchdog did not fire");
            worker.Begin(false);worker.Submit({1},{});
            Wait([&] {return bool(worker.Latest());});
            // Even newly completed inference cannot renew an old input snapshot.
            std::this_thread::sleep_for(Worker::target_timeout+5ms);
            expired=false;
            try {worker.Latest();} catch(const std::runtime_error&) {expired=true;}
            Require(expired,"Stale-target/input watchdog did not fire");
        }
        {
            Worker worker([](const auto&,const auto&) {
                std::this_thread::sleep_for(25ms);return 1;
            },[] {});
            worker.Begin(true);worker.Submit({1},{});
            Wait([&] {
                try {worker.Latest();return false;}
                catch(const std::runtime_error& error) {
                    return std::string(error.what()).find("exceeded 20ms")!=std::string::npos;
                }
            });
        }
        // DWaQ may configure step_dt; the watchdog follows that period too.
        {
            Worker worker([](const auto&,const auto&) {return 1;},[] {},false,10ms);
            worker.Begin(false);
            std::this_thread::sleep_for(35ms);
            bool expired=false;
            try {worker.Latest();} catch(const std::runtime_error&) {expired=true;}
            Require(expired,"Configured-period watchdog did not fire");
        }
        std::cout<<"Policy worker mailbox, reset, exception and watchdog checks passed\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
