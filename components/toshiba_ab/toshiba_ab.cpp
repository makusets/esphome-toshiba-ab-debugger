#include "toshiba_ab.h"
#include "esphome.h"
#ifdef USE_ESP8266
#include <Arduino.h>
#include <esp8266_peri.h>
extern "C" {
#include <ets_sys.h>
}
#endif


namespace esphome {
namespace toshiba_ab {


static uint8_t tx_enable_pin = 16;  // TX enable pin, when low = receive mode, when high = transmit mode

static const char *const TAG = "Toshiba logger";
const LogString *opcode_to_string(uint8_t opcode) {
  switch (opcode) {
    case OPCODE_PING:
      return LOG_STR("OPCODE_PING");
    case OPCODE_PARAMETER:
      return LOG_STR("OPCODE_PARAMETER");
    case OPCODE_ERROR_HISTORY:
      return LOG_STR("OPCODE_ERROR_HISTORY");
    case OPCODE_SENSOR_QUERY:
      return LOG_STR("OPCODE_SENSOR_QUERY");
    case OPCODE_ACK:
      return LOG_STR("OPCODE_ACK");
    case OPCODE_SENSOR_VALUE:
      return LOG_STR("OPCODE_SENSOR_VALUE");
    case OPCODE_STATUS:
      return LOG_STR("OPCODE_STATUS");
    case OPCODE_TEMPERATURE:
      return LOG_STR("OPCODE_TEMPERATURE");
    case OPCODE_EXTENDED_STATUS:
      return LOG_STR("OPCODE_EXTENDED_STATUS");
    default:
      // return LOG_STR(str_sprintf("UNKNOWN OPCODE 1: 0x%02x", opcode));
      return LOG_STR("UNKNOWN");
  }
}

// -------------------------------------------------------------------
// Logging utilities
// -------------------------------------------------------------------



void log_data_frame(const std::string msg, const struct DataFrame *frame, size_t length = 0) {
  std::string res;
  char buf[5];
  size_t len = length > 0 ? length : frame->data_length;
  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      res += ':';
    }
    sprintf(buf, "%02X", frame->data[i]);
    res += buf;
  }
  ESP_LOGD(TAG, "%s: %02X:%02X:\x1B[32m%02X\033[0m:%02X:\033[2;100;37m%s\033[0m:%02X", msg.c_str(), frame->source,
           frame->dest, frame->opcode1, frame->data_length, res.c_str(), frame->crc());
}

void log_raw_data(const std::string& prefix, const uint8_t raw[], size_t size) {
  std::string res;
  res.reserve(size ? (size * 3 - 1) : 0);  // pre-size: "AA:" per byte minus last colon
  char buf[3];
  for (size_t i = 0; i < size; i++) {
    if (i > 0) res += ':';
    std::snprintf(buf, sizeof(buf), "%02X", raw[i]);
    res += buf;
  }
  ESP_LOGD(TAG, "%s%s", prefix.c_str(), res.c_str());
}


void ToshibaAbLogger::dump_config() {
  ESP_LOGCONFIG(TAG, "Owned UART0 receiver: RX GPIO%u, %lu baud, %u data bits, parity %u, %u stop bits",
                rx_pin_, static_cast<unsigned long>(baud_rate_), data_bits_, parity_, stop_bits_);
  ESP_LOGCONFIG(TAG, "Silence timeout: requested %lu us, hardware %lu us; mode: %s",
                static_cast<unsigned long>(frame_gap_us_), static_cast<unsigned long>(effective_gap_us_),
                debug_mode_ == DebugMode::RAW ? "raw" : "normal");
}

void ToshibaAbLogger::setup() {
#ifdef USE_ESP8266
  // Direct register configuration: no ESPHome UART or Arduino Serial receiver.
  ETS_UART_INTR_DISABLE();
  USIE(0) = 0;
  pinMode(tx_enable_pin, OUTPUT);
  digitalWrite(tx_enable_pin, LOW);
  if (rx_pin_ == 13) {
    pinMode(13, FUNCTION_4);
    IOSWAP |= (1U << IOSWAPU0);
  } else {
    pinMode(3, SPECIAL);
    IOSWAP &= ~(1U << IOSWAPU0);
  }
  // Only the RX pin is muxed; TX is never enabled on a GPIO.
  USD(0) = (80000000UL + baud_rate_ / 2) / baud_rate_;
  uint32_t config = (static_cast<uint32_t>(data_bits_ - 5) << UCBN) |
                    (static_cast<uint32_t>(stop_bits_ == 1 ? 1 : 3) << UCSBN);
  if (parity_ != 0)
    config |= (1U << UCPAE) | (parity_ == 2 ? (1U << UCPA) : 0);
  USC0(0) = config | (1U << UCRXRST) | (1U << UCTXRST);
  USC0(0) = config;
  const uint32_t bits = 1 + data_bits_ + (parity_ != 0) + stop_bits_;
  const uint32_t symbols = (static_cast<uint64_t>(frame_gap_us_) * baud_rate_ + bits * 1000000 - 1) /
                           (bits * 1000000);
  effective_gap_us_ = (symbols * bits * 1000000UL + baud_rate_ - 1) / baud_rate_;
  // Interrupt after two bytes. Keep one in the FIFO when draining so the UART
  // silence detector stays armed even during arbitrarily long captures.
  USC1(0) = (2U << UCFFT) | (symbols << UCTOT) | (1U << UCTOE);
  USIC(0) = 0xffff;
  ETS_UART_INTR_ATTACH(uart_isr_, this);
  USIE(0) = (1U << UIFF) | (1U << UITO) | (1U << UIOF) | (1U << UIPE) | (1U << UIFR);
  uart_started_ = true;
  ETS_UART_INTR_ENABLE();
#else
  this->mark_failed();
#endif
  last_master_alive_millis_ = millis();
  metric_window_start_ms_ = millis();
  metric_bucket_start_ms_ = metric_window_start_ms_;
  data_reader.set_noise_counter(debug_mode_ == DebugMode::NORMAL ? &metric_noise_live_ : nullptr);
}

void IRAM_ATTR ToshibaAbLogger::uart_isr_(void *arg) {
#ifdef USE_ESP8266
  auto *self = static_cast<ToshibaAbLogger *>(arg);
  const uint32_t status = USIS(0);
  const uint32_t now_us = micros();
  if (status & (1U << UIOF)) self->capture_.error(CaptureQueue::FIFO_LOSS);
  if (status & (1U << UIPE)) self->capture_.error(CaptureQueue::PARITY_ERROR);
  if (status & (1U << UIFR)) self->capture_.error(CaptureQueue::FRAMING_ERROR);
  const bool idle = status & (1U << UITO);
  uint32_t count = (USS(0) >> USRXC) & 0xff;
  // Snapshot and bound the work; leave one byte unless this is a silence IRQ.
  if (!idle && count > 0) --count;
  for (uint32_t i = 0; i < count; ++i)
    self->capture_.put(static_cast<uint8_t>(USF(0)), now_us);
  if (idle)
    self->capture_.finish(now_us);
  USIC(0) = status;
#endif
}

void ToshibaAbLogger::on_shutdown() {
#ifdef USE_ESP8266
  if (uart_started_) {
    ETS_UART_INTR_DISABLE();
    USIE(0) = 0;
    USC1(0) = 0;
    USIC(0) = 0xffff;
    ETS_UART_INTR_ATTACH(nullptr, nullptr);
    uart_started_ = false;
  }
#endif
}

void ToshibaAbLogger::process_received_data(const struct DataFrame *frame) {
  if (frame->source == this->master_address_) {
      // status update
      ESP_LOGD(TAG, "Received data from master:");
      last_master_alive_millis_ = millis();

      switch (frame->opcode1) {
        case OPCODE_PING: {
        log_data_frame("PING/ALIVE", frame);
        break;
        }
        case OPCODE_ACK: {
        // ACK (maps to 0xA1)
        log_data_frame("ACK", frame);
        break;
      }
        case OPCODE_PARAMETER:
          // master reporting it's state
          // e.g. 01:52:11:04:80:86:A1:05:E4
          /*
          00 52 11 04 80 86 24 00 65  heat
                |-opc1      |  |- mode  bit7-bit5, power bit0, bit2 ???
                            |- 0010 0100 -> mode bit7-bit5  bit4-bit0 ???
                              ---
          */
          log_data_frame("MASTER PARAMETERS", frame);
          {
            const uint8_t modepow = frame->data[STATUS_DATA_MODEPOWER_BYTE];
            const uint8_t power   = (modepow & STATUS_DATA_POWER_MASK);
            const uint8_t mode    = (modepow & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
            ESP_LOGD(TAG, "Decoded PARAMS: power=%u mode=0x%02X", power, mode);
          }

          break;
        case OPCODE_STATUS:
          // sync power, mode, fan and target temp from the unit to the climate
          // component

          log_data_frame("STATUS", frame);

          // this message means that the command sent to master was confirmed
          // (may be it can return an error, but no idea how to read that at the
          // moment)
          {
            const uint8_t modepow = frame->data[STATUS_DATA_MODEPOWER_BYTE];
            const uint8_t power   = (modepow & STATUS_DATA_POWER_MASK);
            const uint8_t mode    = (modepow & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
            const uint8_t fan     = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
            const uint8_t vent    = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;
            const float   tgt     = static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE] & TEMPERATURE_DATA_MASK) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
            ESP_LOGD(TAG, "Decoded STATUS: power=%u mode=0x%02X fan=0x%02X vent=%u target=%.1f°C", power, mode, fan, vent, tgt);
          }

          break;
        case OPCODE_EXTENDED_STATUS:
          // sync power, mode, fan and target temp from the unit to the climate
          // component

          log_data_frame("EXTENDED STATUS", frame);

          {
            const uint8_t modepow = frame->data[STATUS_DATA_MODEPOWER_BYTE];
            const uint8_t power   = (modepow & STATUS_DATA_POWER_MASK);
            const uint8_t mode    = (modepow & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
            const uint8_t fan     = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
            const uint8_t vent    = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;
            const float   tgt     = static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE] & TEMPERATURE_DATA_MASK) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
            float room = NAN;
            if (frame->data[STATUS_DATA_TARGET_TEMP_BYTE + 1] > 1) {
              room = static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE + 1]) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
            }
            const bool preheat  = (frame->data[STATUS_DATA_FLAGS_BYTE] & 0b00000010) >> 1;
            const bool flt      = (frame->data[STATUS_DATA_FLAGS_BYTE] & 0b10000000) >> 7;
            ESP_LOGD(TAG, "Decoded XSTATUS: power=%u mode=0x%02X fan=0x%02X vent=%u target=%.1f°C room=%.1f°C preheat=%d filter=%d",
                     power, mode, fan, vent, tgt, room, preheat, flt);
          }          

          break;
        case OPCODE_SENSOR_VALUE:
            // sensor value received from master
          log_data_frame("SENSOR VALUE (0x1A)", frame);
          break;

        default:
          log_data_frame("MASTER", frame);
          break;
      }
    }else {
    if (frame->source == TOSHIBA_REMOTE) {
    ESP_LOGD(TAG, "Received data from remote:");

    // Remote temperature push: 40 00 55 05 08 81 01 6E 00 ..
    if (frame->opcode1 == OPCODE_TEMPERATURE &&
        frame->data_length >= 4 &&
        frame->data[1] == 0x81) {
      uint8_t raw = frame->data[3] & TEMPERATURE_DATA_MASK;  // raw[7]
      float rmt = static_cast<float>(raw) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
      ESP_LOGD(TAG, "Remote temperature: %.1f°C", rmt);
      log_data_frame("Remote temperature", frame);

    // Remote PING sent every 30s: 40 00 15 07 08 0C 81 00 00 48 00 ..
    } else if (frame->opcode1 == OPCODE_ERROR_HISTORY &&      // 0x15 envelope
              frame->data_length >= 3 &&
              frame->data[0] == COMMAND_MODE_READ &&         // 0x08
              frame->data[1] == OPCODE2_PING_PONG &&         // 0x0C
              frame->data[2] == OPCODE2_READ_STATUS) {       // 0x81
      log_data_frame("Remote PING", frame);
    
    // Remote 40:00:15:06:08:E8:00:01:00:9E:2C that is sent every minute, not sure what it does
    }  else if (frame->opcode1 == OPCODE_ERROR_HISTORY &&      // 0x15 envelope
              frame->data_length >= 6 &&
              frame->data[0] == COMMAND_MODE_READ &&         // 0x08
              frame->data[1] == 0xE8 &&
              frame->data[3] == 0x01 &&                 
              frame->data[5] == 0x9E) {                       
              
      log_data_frame("Remote E8 Read", frame);

    } else {
      // unknown remote message
      log_data_frame("Unknown remote data", frame);
    }
    } else {
      ESP_LOGD(TAG, "Received data from unknown source: %02X", frame->source);
      log_data_frame("Unknown source", frame);
    }
    }
  }

bool ToshibaAbLogger::receive_data(const std::vector<uint8_t> data) {
  auto frame = DataFrame();

  for (size_t i = 0; i < data.size(); i++) {
    frame.raw[i] = data[i];
  }

  return receive_data_frame(&frame);
}

bool ToshibaAbLogger::receive_data_frame(const struct DataFrame *frame) {
  if (frame->crc() != frame->calculate_crc()) {
    ESP_LOGW(TAG, "CRC check failed");
    log_data_frame("Failed frame", frame);
   //Noise  Metrics: bump CRC-fail counter for the current 20 s bucket
    this->metric_failed_live_++;

    return false;
  }
  process_received_data(frame);
  return true;
}

// -------------------------------------------------------------------
// Noise Metrics
void ToshibaAbLogger::metrics_roll_and_log_() {
  const uint32_t now = millis();
  // Roll one or more 20 s buckets if we've crossed boundaries
  while ((now - this->metric_bucket_start_ms_) >= METRIC_BUCKET_MS) {
    // Store the just-finished live counts into the current bucket slot
    this->metric_buckets_[this->metric_idx_].failed_crc = this->metric_failed_live_;
    this->metric_buckets_[this->metric_idx_].noise      = this->metric_noise_live_;
    // Reset live counts for the next 20 s window
    this->metric_failed_live_ = 0;
    this->metric_noise_live_  = 0;
    // Keep the reader writing into the fresh live counter
    this->data_reader.set_noise_counter(&this->metric_noise_live_);
    // Advance time & index
    this->metric_bucket_start_ms_ += METRIC_BUCKET_MS;
    this->metric_idx_ = (this->metric_idx_ + 1) % METRIC_BUCKETS;

    // Compute sums over the last 5 minutes (the 15 stored buckets)
    uint32_t sum_fail = 0, sum_noise = 0;
    for (uint8_t i = 0; i < METRIC_BUCKETS; i++) {
      sum_fail  += this->metric_buckets_[i].failed_crc;
      sum_noise += this->metric_buckets_[i].noise;
    }
    // Work out how many minutes have actually elapsed since start (cap at 5)
    const float elapsed_min = std::min(5.0f, (now - this->metric_window_start_ms_) / 60000.0f);
    if (elapsed_min > 0.0f) {
      const float avg_fail_per_min  = sum_fail  / elapsed_min;
      const float avg_noise_per_min = sum_noise / elapsed_min;
      ESP_LOGI(TAG,
               "AB stats (last %.1f min): CRC fails = %.1f/min, Ignoring noise = %.1f/min "
               "[window totals: fail=%lu, noise=%lu]",
               elapsed_min, avg_fail_per_min, avg_noise_per_min,
               static_cast<unsigned long>(sum_fail),
               static_cast<unsigned long>(sum_noise));
    }
  }
}
// -------------------------------------------------------------------  

void ToshibaAbLogger::log_capture_(const CapturedFrame &frame) {
  const std::string prefix = str_sprintf(
      "RAW %s id=%lu first_irq_us=%lu boundary_irq_us=%lu bytes=%u errors=0x%02X: ",
      frame.final ? "FRAME-END" : "OVERFLOW-FRAGMENT", static_cast<unsigned long>(frame.frame_id),
      static_cast<unsigned long>(frame.first_irq_us), static_cast<unsigned long>(frame.boundary_irq_us),
      frame.length, frame.flags);
  log_raw_data(prefix, frame.data, frame.length);
}

void ToshibaAbLogger::loop() {
  // Bound loop work so logging a busy receiver does not starve other components.
  for (uint8_t n = 0; n < 7; ++n) {
    CapturedFrame frame;
#ifdef USE_ESP8266
    ETS_UART_INTR_DISABLE();
#endif
    const bool have_frame = capture_.pop(frame);
    const uint32_t dropped = capture_.dropped();
#ifdef USE_ESP8266
    ETS_UART_INTR_ENABLE();
#endif
    if (dropped != reported_dropped_) {
      ESP_LOGW(TAG, "Capture queue overflow: %lu records dropped; capture is incomplete",
               static_cast<unsigned long>(dropped - reported_dropped_));
      reported_dropped_ = dropped;
      data_reader.reset();
    }
    if (!have_frame)
      break;
    if (debug_mode_ == DebugMode::RAW) {
      log_capture_(frame);
    } else {
      if (frame.flags) {
        ESP_LOGW(TAG, "UART capture errors: 0x%02X; skipping damaged fragment", frame.flags);
        data_reader.reset();
        continue;
      }
      for (size_t i = 0; i < frame.length; ++i) {
        if (data_reader.put(frame.data[i])) {
          receive_data_frame(&data_reader.frame);
          data_reader.reset();
        }
      }
      if (frame.final) data_reader.reset();
    }
  }
  if (debug_mode_ == DebugMode::NORMAL) {
    if ((millis() - last_master_alive_millis_) > LAST_ALIVE_TIMEOUT_MILLIS) {
      ESP_LOGW(TAG, "No master frames for a while (disconnected?)");
      last_master_alive_millis_ = millis();
    }
    metrics_roll_and_log_();
  }
}

void ToshibaAbLogger::set_debug_mode(const std::string &mode) {
  if (mode == "raw")
    this->debug_mode_ = DebugMode::RAW;
  else
    this->debug_mode_ = DebugMode::NORMAL;
}



}  // namespace toshiba_ab
}  // namespace esphome

void esphome::toshiba_ab::ToshibaAbLogger::set_master_address(uint8_t address) {
  this->master_address_ = address;
}
