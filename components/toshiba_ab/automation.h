#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "tcc_link.h"

namespace esphome
{
  namespace toshiba_ab
  {

    class ToshibaAbOnDataReceivedTrigger : public Trigger<std::vector<uint8_t>>
    {
    public:
      ToshibaAbOnDataReceivedTrigger(ToshibaAbClimate *climate)
      {
        climate->add_on_data_received_callback(
            [this](const struct DataFrame *frame)
            {
              this->trigger(frame->get_data());
            });
      }
    };

  } // namespace toshiba_ab
} // namespace esphome