#pragma once

#include <array>
#include <deque>
#include <stdexcept>

namespace g1_inference {
// 固定长度历史：推理更新时不分配存储，帧顺序始终为从旧到新。
// 首帧复制到整个窗口，避免启动时把尚不存在的历史当作零观测。
// 调用方重置策略时将 initialized 设为 false，下一帧就会覆盖整个窗口。
template<class Frame, std::size_t Count>
void PushHistory(std::array<Frame, Count>& history, const Frame& frame, bool& initialized) {
    static_assert(Count > 0);
    if (!initialized) {
        history.fill(frame);
        initialized = true;
    } else {
        for (std::size_t frame_index = 0; frame_index + 1 < Count; ++frame_index) {
            history[frame_index] = history[frame_index + 1];
        }
        history.back() = frame;
    }
}

// 相机模式的历史长度由运行时配置决定，使用 deque；同样从旧到新保存。
// 重置时由调用方清空队列；已有历史期间不允许改变窗口长度。
template<class Frame>
void PushHistory(std::deque<Frame>& history, const Frame& frame, std::size_t capacity) {
    if (capacity == 0) throw std::runtime_error("History requires a positive capacity");
    if (history.empty()) {
        history.assign(capacity, frame);
    } else {
        if (history.size() != capacity) {
            throw std::runtime_error("History capacity changed; restart policy");
        }
        history.pop_front();
        history.push_back(frame);
    }
}

// 按训练时的 term-major 布局展开：先输出某一观测项的全部历史，再输出下一项。
// 例如两帧 [角速度0, 重力0]、[角速度1, 重力1] 展开为
// [角速度0, 角速度1, 重力0, 重力1]，每项内部仍按从旧到新的时间顺序排列。
// output 可指向固定数组或插入迭代器；调用方保证能容纳所有帧的元素。
template<class History, std::size_t Terms, class OutputIterator>
void PackTermMajor(const History& history, const std::array<int, Terms>& term_sizes, OutputIterator output) {
    if (history.empty()) throw std::runtime_error("Cannot pack empty observation history");
    std::size_t frame_size = 0;
    for (int term_size : term_sizes) {
        if (term_size <= 0) throw std::runtime_error("Observation terms require positive sizes");
        frame_size += static_cast<std::size_t>(term_size);
    }
    if (frame_size != history.front().size()) {
        throw std::runtime_error("Observation term sizes do not match the frame");
    }
    int term_offset = 0;
    for (int term_size : term_sizes) {
        for (const auto& frame : history) {
            for (int component = 0; component < term_size; ++component) {
                *output = frame[term_offset + component];
                ++output;
            }
        }
        term_offset += term_size;
    }
}
}  // namespace g1_inference
