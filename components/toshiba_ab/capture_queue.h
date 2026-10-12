#pragma once
#include <cstddef>
#include <cstdint>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

namespace esphome { namespace toshiba_ab {

// Single ISR producer, single loop consumer. The consumer masks UART interrupts
// while copying a slot. All ISR storage is fixed, and ISR methods never log.
struct CapturedFrame {
  uint8_t data[128]{};
  uint16_t length{0};
  uint16_t flags{0};
  uint32_t frame_id{0};
  uint32_t first_irq_us{0};
  uint32_t boundary_irq_us{0};
  bool final{false};
};

class CaptureQueue {
 public:
  static constexpr uint16_t FIFO_LOSS = 1, PARITY_ERROR = 2, FRAMING_ERROR = 4, QUEUE_LOSS = 8;
  void IRAM_ATTR error(uint16_t flags) { active_.flags |= flags; }
  void IRAM_ATTR put(uint8_t byte, uint32_t irq_us) {
    if (active_.length == sizeof(active_.data))
      publish_(false, irq_us);
    if (!started_) {
      active_.first_irq_us = irq_us;
      active_.frame_id = frame_id_;
      started_ = true;
    }
    active_.data[active_.length++] = byte;
  }
  void IRAM_ATTR finish(uint32_t irq_us) {
    if (!started_)
      return;
    publish_(true, irq_us);
    active_.flags = 0;
    started_ = false;
    ++frame_id_;
  }
  // Call only with UART IRQ masked. Logging happens after it is unmasked.
  bool pop(CapturedFrame &out) {
    if (tail_ == head_)
      return false;
    out = slots_[tail_];
    tail_ = (tail_ + 1) % SLOT_COUNT;
    return true;
  }
  uint32_t dropped() const { return dropped_; }
  bool idle() const { return !started_ && head_ == tail_; }
  // Receiver is idle and its IRQ is masked when resetting a trial.
  void reset_trial() {
    tail_ = head_; active_.length = 0; active_.flags = 0; started_ = false;
  }

 private:
  void IRAM_ATTR publish_(bool final, uint32_t irq_us) {
    const uint8_t next = (head_ + 1) % SLOT_COUNT;
    if (next == tail_) {
      ++dropped_;
      loss_pending_ = true;
    } else {
      CapturedFrame &slot = slots_[head_];
      slot.length = active_.length;
      slot.flags = active_.flags | (loss_pending_ ? QUEUE_LOSS : 0);
      slot.frame_id = active_.frame_id;
      slot.first_irq_us = active_.first_irq_us;
      slot.boundary_irq_us = irq_us;
      slot.final = final;
      for (uint16_t i = 0; i < active_.length; ++i)
        slot.data[i] = active_.data[i];
      __asm__ __volatile__("" ::: "memory");
      head_ = next;
      loss_pending_ = false;
    }
    active_.length = 0;
  }
  static constexpr uint8_t SLOT_COUNT = 8;  // Seven queued records + producer slot.
  CapturedFrame slots_[SLOT_COUNT]{};
  CapturedFrame active_{};
  volatile uint8_t head_{0}, tail_{0};
  volatile uint32_t dropped_{0};
  uint32_t frame_id_{0};
  bool started_{false}, loss_pending_{false};
};

}}  // namespace esphome::toshiba_ab
