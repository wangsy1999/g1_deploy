#pragma once

#include <array>
#include <deque>
#include <stdexcept>

namespace g1_inference {
// Fixed storage for short state histories; no allocation during policy updates.
template<class Frame, std::size_t Count>
void PushHistory(std::array<Frame, Count>& history, const Frame& frame, bool& initialized) {
    static_assert(Count > 0);
    if (!initialized) {
        history.fill(frame);
        initialized = true;
    } else {
        for (std::size_t i = 0; i + 1 < Count; ++i) history[i] = history[i + 1];
        history.back() = frame;
    }
}

// Runtime capacity supports the camera mode's longer image histories.
template<class Frame>
void PushHistory(std::deque<Frame>& history, const Frame& frame, std::size_t capacity) {
    if (capacity == 0) throw std::runtime_error("History requires a positive capacity");
    if (history.empty()) {
        history.assign(capacity, frame);
    } else {
        if (history.size() != capacity)
            throw std::runtime_error("History capacity changed; restart policy");
        history.pop_front();
        history.push_back(frame);
    }
}

// Training layout: all time samples for one term, then the next term.
// Frames are kept oldest -> newest. Output can be a fixed array or an inserter.
template<class History, std::size_t Terms, class OutputIterator>
void PackTermMajor(const History& history, const std::array<int, Terms>& sizes, OutputIterator output) {
    if (history.empty()) throw std::runtime_error("Cannot pack empty observation history");
    std::size_t frame_size = 0;
    for (int size : sizes) {
        if (size <= 0) throw std::runtime_error("Observation terms require positive sizes");
        frame_size += static_cast<std::size_t>(size);
    }
    if (frame_size != history.front().size())
        throw std::runtime_error("Observation term sizes do not match the frame");
    int offset = 0;
    for (int size : sizes) {
        for (const auto& frame : history)
            for (int i = 0; i < size; ++i) *output++ = frame[offset + i];
        offset += size;
    }
}
}  // namespace g1_inference
