#pragma once
#include "esphome/core/component.h"
#define INPUT_PULLUP 5
#define OUTPUT 1
#define LOW 0
#define SPECIAL 2
#define FUNCTION_4 3
inline int configured_pin=-1, configured_mode=-1;
inline void pinMode(uint8_t pin,int mode) { configured_pin=pin; configured_mode=mode; }
inline void digitalWrite(uint8_t,int) {}

#define HIGH 1
#define CHANGE 4
inline int rx_level = HIGH;
inline int gpio_read_level = -1;
inline int digitalRead(uint8_t) { return configured_mode == INPUT_PULLUP || gpio_read_level < 0 ? rx_level : gpio_read_level; }
inline void (*edge_callback)(void *)=nullptr;
inline void *edge_arg=nullptr;
inline void attachInterruptArg(uint8_t, void (*f)(void *), void *a, int) { edge_callback=f; edge_arg=a; }
inline void detachInterrupt(uint8_t) { edge_callback=nullptr; edge_arg=nullptr; }
