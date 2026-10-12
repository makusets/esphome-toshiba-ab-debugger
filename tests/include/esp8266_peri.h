#pragma once
#include "esphome/core/component.h"
inline std::queue<uint8_t> fifo;
struct FifoRead {
  operator uint8_t() const { assert(!fifo.empty()); auto b=fifo.front(); fifo.pop(); return b; }
};
inline FifoRead fifo_read;
inline uint32_t status_reg=0, enabled_reg=0, cleared_reg=0, clock_reg=0, conf0_reg=0, conf1_reg=0, swap_reg=0;
#define USF(u) fifo_read
#define USIS(u) status_reg
#define USIE(u) enabled_reg
#define USIC(u) cleared_reg
#define USD(u) clock_reg
#define USC0(u) conf0_reg
#define USC1(u) conf1_reg
#define USS(u) (static_cast<uint32_t>(fifo.size()) | (rx_level == HIGH ? (1U << 15) : 0))
#define IOSWAP swap_reg
#define IOSWAPU0 2
#define UIFF 0
#define UIPE 2
#define UIFR 3
#define UIOF 4
#define UITO 8
#define USRXC 0
#define UCPAE 1
#define UCPA 0
#define UCBN 2
#define UCSBN 4
#define UCRXRST 17
#define UCTXRST 18
#define UCTOE 31
#define UCTOT 24
#define UCFFT 0

#define USRXD 15
