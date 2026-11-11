#pragma once

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_joints {
inline const std::array<std::string, 29> kDdsNames{
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};

template<std::size_t N> void ValidatePermutation(const std::array<int, N>& ids) {
    std::array<bool, N> seen{};
    for (int id : ids) {
        if (id < 0 || id >= static_cast<int>(N) || seen[id])
            throw std::runtime_error("Joint mapping must be a permutation of motor indices");
        seen[id] = true;
    }
}
inline std::array<int, 29> FromNames(const std::vector<std::string>& names) {
    if (names.size() != kDdsNames.size()) throw std::runtime_error("Expected 29 policy joints");
    std::array<int, 29> ids{};
    for (std::size_t i = 0; i < names.size(); ++i) {
        const auto found = std::find(kDdsNames.begin(), kDdsNames.end(), names[i]);
        if (found == kDdsNames.end()) throw std::runtime_error("Unknown joint: " + names[i]);
        ids[i] = static_cast<int>(found - kDdsNames.begin());
    }
    ValidatePermutation(ids);
    return ids;
}
// Mappings are validated once during configuration, not in the control loop.
template<class T, std::size_t N>
std::array<T, N> ToMotorOrder(const std::array<T, N>& values, const std::array<int, N>& ids) {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) result[ids[i]] = values[i];
    return result;
}
template<class T, std::size_t N>
std::array<T, N> ToMotorOrder(const std::array<T, N>& values, const std::array<int, N>& ids,
                             const std::array<T, N>& signs) {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) result[ids[i]] = signs[i] * values[i];
    return result;
}
}  // namespace g1_joints
