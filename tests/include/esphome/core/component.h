#pragma once
#include <algorithm>
#include <bitset>
#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <queue>
#include <string>
#include <vector>
#define IRAM_ATTR
inline uint32_t fake_now = 0;
inline bool in_irq = false;
inline std::vector<std::string> logs;
inline uint32_t micros() { return fake_now; }
inline uint32_t millis() { return fake_now / 1000; }
inline std::string str_sprintf(const char *fmt, ...) {
  char buf[2048]; va_list ap; va_start(ap,fmt);
  vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap); return buf;
}
inline void test_log(const char *, const char *fmt, ...) {
  assert(!in_irq);
  char buf[2048]; va_list ap; va_start(ap,fmt);
  vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap); logs.emplace_back(buf);
}
namespace esphome {
struct Component {
  virtual void setup() {} virtual void loop() {} virtual void dump_config() {}
  virtual void on_shutdown() {} void mark_failed() {}
};
}
