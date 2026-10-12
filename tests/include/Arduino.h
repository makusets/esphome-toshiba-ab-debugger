#pragma once
#include "esphome/core/component.h"
#define OUTPUT 1
#define LOW 0
#define SPECIAL 2
#define FUNCTION_4 3
inline int configured_pin=-1, configured_mode=-1;
inline void pinMode(uint8_t pin,int mode) { configured_pin=pin; configured_mode=mode; }
inline void digitalWrite(uint8_t,int) {}
