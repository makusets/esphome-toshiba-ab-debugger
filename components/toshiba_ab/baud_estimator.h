#pragma once
#include <cstddef>
#include <cstdint>
#include "capture_queue.h"

namespace esphome { namespace toshiba_ab {

// GPIO ISR producer, loop consumer. Mask GPIO IRQ while popping/discarding.
class RxEdgeIntervals {
 public:
  void IRAM_ATTR put(uint32_t now_us) {
    const uint32_t delta = now_us - previous_us_;
    previous_us_ = now_us;
    if (!seen_) { seen_ = true; return; }
    // Ignore zero-duration callbacks and long inter-burst idle periods.
    if (delta == 0 || delta >= 50000) return;
    const uint16_t next = (head_ + 1) & 255;
    if (next == tail_) { ++dropped_; return; }
    intervals_[head_] = delta;
    __asm__ __volatile__("" ::: "memory");
    head_ = next;
  }
  bool pop(uint32_t &interval) {
    if (tail_ == head_) return false;
    interval = intervals_[tail_]; tail_ = (tail_ + 1) & 255;
    return true;
  }
  uint32_t dropped() const { return dropped_; }
  void discard() { tail_ = head_; }
 private:
  uint32_t intervals_[256]{}, previous_us_{0};
  volatile uint16_t head_{0}, tail_{0};
  volatile uint32_t dropped_{0};
  bool seen_{false};
};

// Timing suggestions only: no UART configuration is changed here. Arithmetic,
// ranking, and text formatting run outside the interrupt handler.
class BaudEstimator {
 public:
  void set_configured(uint32_t baud) {
    bool present = false;
    for (size_t i = 0; i < 10; ++i) present |= rates_[i] == baud;
    rate_count_ = present ? 10 : 11;
    if (!present) rates_[10] = baud;
  }
  void reset() {
    total_ = 0;
    for (size_t i = 0; i < 11; ++i) matches_[i] = singles_[i] = 0;
  }
  void observe(uint32_t interval_us) {
    if (interval_us == 0 || interval_us >= 50000 || total_ >= 4096) return;
    ++total_;
    for (size_t i = 0; i < rate_count_; ++i) {
      const uint64_t scaled = static_cast<uint64_t>(interval_us) * rates_[i];
      const uint32_t bits = (scaled + 500000) / 1000000;
      if (bits < 1 || bits > 12) continue;
      const uint64_t target = static_cast<uint64_t>(bits) * 1000000;
      const uint64_t error = scaled > target ? scaled - target : target - scaled;
      // 12% of one bit, with a 4 us floor for timestamp/interrupt jitter.
      const uint32_t tolerance = rates_[i] * 4 > 120000 ? rates_[i] * 4 : 120000;
      if (error <= tolerance) {
        ++matches_[i]; if (bits == 1) ++singles_[i];
      }
    }
  }
  bool enough() const { return total_ >= 64; }
  size_t candidates(uint32_t out[3]) const {
    if (!enough()) return 0;
    uint8_t selected[3]{}; size_t count = 0;
    for (size_t n = 0; n < 3; ++n) {
      int best = -1;
      for (size_t i = 0; i < rate_count_; ++i) {
        bool used = false;
        for (size_t j = 0; j < count; ++j) used |= selected[j] == i;
        if (used || matches_[i] * 10 < total_ * 9) continue;
        if (best < 0 || singles_[i] > singles_[best] ||
            (singles_[i] == singles_[best] && matches_[i] > matches_[best]) ||
            (singles_[i] == singles_[best] && matches_[i] == matches_[best] && rates_[i] < rates_[best])) best = i;
      }
      // Need repeated near-single-bit intervals for the best candidate. Faster
      // harmonic fits may still be plausible; include them as alternatives.
      if (best < 0 || (count == 0 && singles_[best] < 8)) break;
      selected[count] = best; out[count++] = rates_[best];
    }
    return count;
  }
 private:
  uint32_t rates_[11]{300,600,1200,2400,4800,9600,19200,38400,57600,115200,0};
  size_t rate_count_{10};
  uint32_t matches_[11]{}, singles_[11]{}, total_{0};
};

}}  // namespace esphome::toshiba_ab
