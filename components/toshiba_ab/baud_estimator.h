#pragma once
#include <cstddef>
#include <cstdint>
#include "capture_queue.h"

namespace esphome { namespace toshiba_ab {

// GPIO ISR producer, loop consumer. Mask GPIO IRQ while popping/discarding.
class RxEdgeIntervals {
 public:
  void IRAM_ATTR put(uint32_t now_us) {
    ++edges_;
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
  uint32_t edges() const { return edges_; }
  void discard() { tail_ = head_; }
 private:
  uint32_t intervals_[256]{}, previous_us_{0};
  volatile uint16_t head_{0}, tail_{0};
  volatile uint32_t dropped_{0}, edges_{0};
  bool seen_{false};
};

// Timing suggestions only: no UART configuration is changed here. Arithmetic,
// ranking, and text formatting run outside the interrupt handler.
class BaudEstimator {
 public:
  void set_configured(uint32_t baud) {
    for (size_t i = 0; i < 10; ++i) {
      if (rates_[i] == baud) { configured_index_ = i; rate_count_ = 10; return; }
    }
    rates_[10] = baud; configured_index_ = 10; rate_count_ = 11;
  }
  void reset() {
    total_ = 0;
    for (size_t i = 0; i < 11; ++i) {
      matches_[i] = singles_[i] = considered_[i] = too_short_[i] = 0;
      for (size_t j = 0; j < 12; ++j) runs_[i][j] = 0;
    }
  }
  void observe(uint32_t interval_us) {
    if (interval_us == 0 || interval_us >= 50000 || total_ >= 4096) return;
    ++total_;
    for (size_t i = 0; i < rate_count_; ++i) {
      const uint64_t scaled = static_cast<uint64_t>(interval_us) * rates_[i];
      const uint32_t bits = (scaled + 500000) / 1000000;
      if (bits == 0) { ++too_short_[i]; continue; }
      // Longer intervals may be idle gaps, not failed bit timing. Score only
      // the 1..12-bit range, requiring enough in-range evidence per candidate.
      if (bits > 12) continue;
      ++considered_[i];
      const uint64_t target = static_cast<uint64_t>(bits) * 1000000;
      const uint64_t error = scaled > target ? scaled - target : target - scaled;
      // 12% of one bit, with a 4 us floor for timestamp/interrupt jitter.
      const uint32_t tolerance = rates_[i] * 4 > 120000 ? rates_[i] * 4 : 120000;
      if (error <= tolerance) {
        ++matches_[i]; if (bits == 1) ++singles_[i];
        if (runs_[i][bits - 1] < 8) ++runs_[i][bits - 1];
      }
    }
  }
  bool enough() const { return total_ >= 64; }
  uint32_t samples() const { return total_; }
  uint32_t configured_short_pulses() const { return too_short_[configured_index_]; }
  void configured_fit(uint32_t &matches, uint32_t &considered, uint32_t &singles) const {
    matches = matches_[configured_index_]; considered = considered_[configured_index_];
    singles = singles_[configured_index_];
  }
  size_t candidates(uint32_t out[3]) const {
    if (!enough()) return 0;
    uint8_t selected[3]{}; size_t count = 0;
    for (size_t n = 0; n < 3; ++n) {
      int best = -1;
      for (size_t i = 0; i < rate_count_; ++i) {
        bool used = false;
        for (size_t j = 0; j < count; ++j) used |= selected[j] == i;
        if (used || considered_[i] < 64 || matches_[i] * 10 < considered_[i] * 9 ||
            too_short_[i] * 10 > total_ || (count == 0 && !fundamental_(i))) continue;
        if (best < 0 || singles_[i] > singles_[best] ||
            (singles_[i] == singles_[best] && matches_[i] > matches_[best]) ||
            (singles_[i] == singles_[best] && matches_[i] == matches_[best] && rates_[i] < rates_[best])) best = i;
      }
      // The best candidate needs a supported fundamental timing grid. Faster
      // harmonic fits may still be plausible; include them as alternatives.
      if (best < 0) break;
      selected[count] = best; out[count++] = rates_[best];
    }
    return count;
  }
 private:
  bool fundamental_(size_t index) const {
    uint32_t divisor = 0;
    for (uint32_t bits = 1; bits <= 12; ++bits) {
      if (runs_[index][bits - 1] < 8) continue;
      uint32_t run = bits;
      while (run != 0) { const uint32_t next = divisor % run; divisor = run; run = next; }
    }
    // Recurring 2-bit and 9-bit runs imply one common bit time even without
    // observing a 1-bit pulse. Requiring eight of each prevents rare outliers
    // from creating an apparently finer timing grid.
    return divisor == 1;
  }
  uint8_t runs_[11][12]{};
  uint32_t rates_[11]{300,600,1200,2400,4800,9600,19200,38400,57600,115200,0};
  size_t rate_count_{10};
  uint32_t matches_[11]{}, singles_[11]{}, considered_[11]{}, too_short_[11]{}, total_{0};
  uint8_t configured_index_{3};
};

}}  // namespace esphome::toshiba_ab
