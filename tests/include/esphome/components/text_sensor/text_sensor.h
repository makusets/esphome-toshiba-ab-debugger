#pragma once
#include "esphome/core/component.h"
namespace esphome { namespace text_sensor {
struct TextSensor {
  std::string state;
  std::vector<std::string> history;
  void publish_state(const std::string &s) { assert(!in_irq); state=s; history.push_back(s); }
};
}}
