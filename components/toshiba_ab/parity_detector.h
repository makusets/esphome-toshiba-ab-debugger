#pragma once
#include <cstdint>
#include "capture_queue.h"

namespace esphome { namespace toshiba_ab {

struct ParitySample {
  uint32_t bytes{0}, frames{0};
  uint32_t parity_events{0}, framing_events{0};  // IRQ events, not damaged-byte counts.
  uint32_t parity_frames{0}, framing_frames{0}, losses{0};
};

// Main-loop policy only. UART configuration and baud are owned by the caller.
class ParityDetector {
 public:
  enum class Action { NONE, NEXT, RETRY, LOCK, UNCERTAIN };
  void start(uint32_t now_ms) {
    phase_ = Phase::SCAN; index_ = 0; parity_ = 1; window_start_ = now_ms;
    reset_evidence();
  }
  uint8_t parity() const { return parity_; }
  bool locked() const { return phase_ == Phase::LOCKED; }
  bool scanning() const { return phase_ == Phase::SCAN; }
  void observe(const CapturedFrame &frame) {
    frame_bad_ |= frame.flags != 0;
    for (uint16_t i = 0; i < frame.length; ++i) {
      hash_ = (hash_ ^ frame.data[i]) * 16777619UL;
      ++length_;
    }
    if (!frame.final) return;
    if (!frame_bad_) {
      bool seen = false;
      for (uint8_t i = 0; i < signature_count_; ++i)
        seen |= signatures_[i] == hash_ && lengths_[i] == length_;
      if (seen) ++repeats_;
      else {
        signatures_[signature_slot_] = hash_; lengths_[signature_slot_] = length_;
        signature_slot_ = (signature_slot_ + 1) % 4;
        if (signature_count_ < 4) ++signature_count_;
      }
    }
    hash_ = 2166136261UL; length_ = 0; frame_bad_ = false;
  }
  Action step(uint32_t now_ms, const ParitySample &sample) {
    const uint32_t elapsed = now_ms - window_start_;
    if (phase_ != Phase::SCAN) return Action::NONE;
    if (elapsed < 20000 || !enough(sample)) return Action::NONE;
    if (sample.losses) {
      window_start_ = now_ms; reset_evidence(); return Action::RETRY;
    }
    samples_[index_] = sample;
    if (index_ < 2) {
      ++index_; parity_ = index_ == 1 ? 2 : 0;
      window_start_ = now_ms; reset_evidence(); return Action::NEXT;
    }
    // NONE cannot win just because it cannot report parity errors. Require
    // both parity modes to fail and repeated clean NONE captures as support.
    int winner = -1;
    if (clean(samples_[0]) && parity_bad(samples_[1]) && framing_bad(samples_[2])) winner = 1;
    if (clean(samples_[1]) && parity_bad(samples_[0]) && framing_bad(samples_[2])) winner = 2;
    if (clean(samples_[2]) && parity_bad(samples_[0]) && parity_bad(samples_[1]) && repeats_ >= 2) winner = 0;
    window_start_ = now_ms; reset_evidence();
    if (winner >= 0) {
      parity_ = winner; phase_ = Phase::LOCKED; return Action::LOCK;
    }
    parity_ = 1; phase_ = Phase::UNCERTAIN; return Action::UNCERTAIN;
  }

 private:
  enum class Phase { SCAN, LOCKED, UNCERTAIN };
  static bool enough(const ParitySample &s) { return s.frames >= 3 && s.bytes >= 12; }
  static bool clean(const ParitySample &s) {
    return s.losses == 0 && static_cast<uint64_t>(s.parity_frames) * 10 <= s.frames &&
           static_cast<uint64_t>(s.framing_frames) * 10 <= s.frames;
  }
  static bool parity_bad(const ParitySample &s) {
    return s.parity_events > 0 && static_cast<uint64_t>(s.parity_frames) * 2 >= s.frames;
  }
  static bool framing_bad(const ParitySample &s) {
    return s.framing_events > 0 && static_cast<uint64_t>(s.framing_frames) * 2 >= s.frames;
  }
  void reset_evidence() {
    signature_count_ = signature_slot_ = 0; repeats_ = 0;
    hash_ = 2166136261UL; length_ = 0; frame_bad_ = false;
  }
  Phase phase_{Phase::SCAN};
  uint8_t index_{0}, parity_{1};
  uint32_t window_start_{0};
  ParitySample samples_[3]{};
  uint32_t signatures_[4]{}, lengths_[4]{};
  uint8_t signature_count_{0}, signature_slot_{0};
  uint32_t repeats_{0}, hash_{2166136261UL}, length_{0};
  bool frame_bad_{false};
};

}}  // namespace esphome::toshiba_ab
