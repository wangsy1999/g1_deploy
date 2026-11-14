#pragma once
#include "g1_parkour_policy.h"
#include "g1_depth_camera.h"
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>

namespace g1_parkour {
using DepthFrame = g1_camera::DepthFrame;
// 同机同用户的最新帧邮箱。生产者独占 .owner 锁，读者只接受活跃生产者的数据。
// 文件锁覆盖帧头和数据的整次复制；锁忙或序号未变时跳过，不阻塞控制循环。
class SharedInput {
    std::string path_;
    std::size_t size_;
    int fd_ = -1;
    int owner_fd_ = -1;
    int reader_fd_ = -1;
    std::array<unsigned char, 32> last_header_{};
    bool have_header_ = false;
public:
    SharedInput(const std::string& kind, std::size_t size, const std::string& directory = "/dev/shm")
        : path_(directory + "/g1-" + kind + "-" + std::to_string(getuid())),
          size_(size) {
        // 与原来的独占接收端口一样，每类输入只允许一个控制器消费。
        reader_fd_ = open((path_ + ".reader").c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (reader_fd_ < 0) throw std::runtime_error("Cannot open shared input reader lock");
        if (flock(reader_fd_, LOCK_EX | LOCK_NB)) {
            close(reader_fd_);
            throw std::runtime_error("Shared input already has a controller");
        }
    }
    ~SharedInput() {
        if (fd_ >= 0) close(fd_);
        if (owner_fd_ >= 0) close(owner_fd_);
        if (reader_fd_ >= 0) close(reader_fd_);
    }
    SharedInput(const SharedInput&) = delete;
    SharedInput& operator=(const SharedInput&) = delete;

    template<class Consume> bool Latest(Consume consume) {
        if (owner_fd_ < 0) {
            owner_fd_ = open((path_ + ".owner").c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
            if (owner_fd_ < 0) {
                if (errno == ENOENT) return false; // 允许控制器先于生产者启动。
                throw std::runtime_error("Cannot open shared input owner");
            }
        }
        if (flock(owner_fd_, LOCK_SH | LOCK_NB) == 0) {
            flock(owner_fd_, LOCK_UN);
            return false; // 生产者已退出，不能把残留文件当作新观测。
        }
        if (errno != EWOULDBLOCK) throw std::runtime_error("Cannot check shared input owner");
        if (fd_ < 0) {
            fd_ = open(path_.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
            if (fd_ < 0) {
                if (errno == ENOENT) return false;
                throw std::runtime_error("Cannot open shared input");
            }
        }
        if (flock(fd_, LOCK_SH | LOCK_NB)) {
            if (errno == EWOULDBLOCK) return false;
            throw std::runtime_error("Cannot lock shared input");
        }
        struct Unlock {
            int fd;
            ~Unlock() { if (fd >= 0) flock(fd, LOCK_UN); }
        } unlock{fd_};
        struct stat status{};
        if (fstat(fd_, &status) || status.st_nlink == 0 || status.st_size != static_cast<off_t>(size_)) {
            throw std::runtime_error("Shared input replaced or wrong size; restart controller");
        }
        std::array<unsigned char, 32> header{};
        if (pread(fd_, header.data(), header.size(), 0) != 32) {
            throw std::runtime_error("Incomplete shared input header");
        }
        if (header[0] == 0 || (have_header_ && header == last_header_)) return false;
        std::vector<unsigned char> packet(size_);
        if (pread(fd_, packet.data(), packet.size(), 0) != static_cast<ssize_t>(packet.size())) {
            throw std::runtime_error("Incomplete shared input frame");
        }
        // 复制完即可释放文件锁，解码和消费不占用生产者的写入时间。
        flock(fd_, LOCK_UN);
        unlock.fd = -1;
        consume(packet.data(), packet.size());
        last_header_ = header;
        have_header_ = true;
        return true;
    }
};

// The policy root is torso. LowState's pelvis IMU is deliberately not used here.
inline Observation HardwareObservation(const Policy& policy,
    const unitree_hg::msg::dds_::LowState_& state,
    const unitree_hg::msg::dds_::IMUState_& torso, const DepthFrame& depth) {
    Observation obs;
    obs.raw_depth=depth.raw_depth;
    obs.sequence=depth.sequence; obs.time=depth.captured_at;
    for(int i=0;i<29;++i) {
        obs.q[i]=policy.sign[i]*state.motor_state()[policy.ids[i]].q();
        obs.dq[i]=policy.sign[i]*state.motor_state()[policy.ids[i]].dq();
    }
    const auto& q=torso.quaternion();
    double norm=0; for(float v:q) norm+=double(v)*v;
    if(!std::isfinite(norm) || norm<.5 || norm>1.5)
        throw std::runtime_error("Invalid torso IMU quaternion");
    obs.gravity={float(2*(double(q[0])*q[2]-double(q[1])*q[3])/norm),
                 float(-2*(double(q[0])*q[1]+double(q[2])*q[3])/norm),
                 float((2*(double(q[1])*q[1]+double(q[2])*q[2])-norm)/norm)};
    std::copy_n(torso.gyroscope().begin(),3,obs.omega.begin());
    for(float v:obs.omega) if(!std::isfinite(v)) throw std::runtime_error("Invalid torso gyro");
    // Absolute torso height is not observable from this IMU/LowState input.
    // The controller does not use absolute torso height as a simulation stop condition.
    obs.height=0;
    return obs;
}
} // namespace g1_parkour
