#pragma once

#include "esphome/components/uart/uart.h"

#include <bitset>
#include <queue>
#include <vector>

namespace esphome {
namespace toshiba_ab {

const uint32_t ALIVE_MESSAGE_PERIOD_MILLIS = 5000;
const uint32_t LAST_ALIVE_TIMEOUT_MILLIS = ALIVE_MESSAGE_PERIOD_MILLIS * 3 + 1000;

const uint32_t PACKET_MIN_WAIT_MILLIS = 200;


// const uint8_t TOSHIBA_MASTER = 0x00;  replaced by master_address_ which is setup in yaml
const uint8_t TOSHIBA_REMOTE = 0x40;
const uint8_t TOSHIBA_BROADCAST = 0xFE;
const uint8_t TOSHIBA_REPORT = 0x52;

const uint8_t OPCODE_PING = 0x10;  
const uint8_t OPCODE_PARAMETER = 0x11;
const uint8_t OPCODE_ERROR_HISTORY = 0x15;
const uint8_t OPCODE_SENSOR_QUERY = 0x17;
const uint8_t OPCODE_ACK = 0x18;
const uint8_t OPCODE_SENSOR_VALUE = 0x1A;
const uint8_t OPCODE_STATUS = 0x1C;
const uint8_t OPCODE_TEMPERATURE = 0x55;
const uint8_t OPCODE_EXTENDED_STATUS = 0x58;

const uint8_t STATUS_DATA_MODEPOWER_BYTE = 2;
const uint8_t STATUS_DATA_POWER_MASK = 0b00000001;
const uint8_t STATUS_DATA_MODE_MASK = 0b11100000;
const uint8_t STATUS_DATA_MODE_SHIFT_BITS = 5;
const uint8_t STATUS_DATA_FLAGS_BYTE = 4;
// const uint8_t STATUS_DATA_COOLING_MASK = 0b00001000; // e.g. 48 in
// 01:52:11:04:80:86:48:01:09 const uint8_t STATUS_DATA_COOLING_SHIFT_BITS = 3;
// const uint8_t STATUS_DATA_HEATING_MASK = 0b00000001; // e.g. 81 in
// 01:52:11:04:80:86:81:01:C0. Also in DRY mode
const uint8_t STATUS_DATA_FANVENT_BYTE = 3;
const uint8_t STATUS_DATA_FAN_MASK = 0b11100000;
const uint8_t STATUS_DATA_FAN_SHIFT_BITS = 5;
const uint8_t STATUS_DATA_VENT_MASK = 0b00000100;
const uint8_t STATUS_DATA_VENT_SHIFT_BITS = 2;
const uint8_t STATUS_DATA_TARGET_TEMP_BYTE = 6;

const uint8_t COMMAND_MODE_READ = 0x08;
const uint8_t COMMAND_MODE_WRITE = 0x80;

const uint8_t COMMAND_SET_TEMP = 0b00001000;
const uint8_t COMMAND_SET_FAN = 0b00010000;

const uint8_t COMMAND_MODE_FLAGS_TEMP = 0b00001000;
const uint8_t COMMAND_MODE_FLAGS_FAN = 0b00010000;

const uint8_t OPCODE2_READ_STATUS = 0x81;
const uint8_t OPCODE2_READ_ALIVE = 0x8A;
const uint8_t OPCODE2_PARAM_ACK = 0xA1;
const uint8_t OPCODE2_READ_MODE = 0x86;
const uint8_t OPCODE2_PING_PONG = 0x0C;
const uint8_t OPCODE2_SET_POWER = 0x41;
const uint8_t OPCODE2_SET_MODE = 0x42;
const uint8_t OPCODE2_SET_TEMP_WITH_FAN = 0x4C;
const uint8_t OPCODE2_SET_VENT = 0x52;
const uint8_t OPCODE2_SAVE = 0x54;
const uint8_t OPCODE2_SENSOR_QUERY = 0x80;
const uint8_t OPCODE2_SENSOR_ROOM_TEMP = 0x81;

const uint8_t TEMPERATURE_DATA_MASK = 0b11111110;
const float TEMPERATURE_CONVERSION_RATIO = 2.0;
const float TEMPERATURE_CONVERSION_OFFSET = 35.0;

const uint8_t EMPTY_DATA = 0x00;

const uint8_t POWER_ON = 0x01;
const uint8_t POWER_OFF = 0x00;

const uint8_t MODE_MASK = 0x07;
const uint8_t MODE_HEAT = 0x01;
const uint8_t MODE_COOL = 0x02;
const uint8_t MODE_FAN_ONLY = 0x03;
const uint8_t MODE_DRY = 0x04;
const uint8_t MODE_AUTO = 0x05;

const uint8_t FAN_PEED_AUTO = 0x02;
const uint8_t FAN_PEED_LOW = 0x05;
const uint8_t FAN_PEED_MED = 0x04;
const uint8_t FAN_PEED_HIGH = 0x03;

const uint8_t SET_PARAMETER_PAYLOAD_HEADER_SIZE = 2;

const uint8_t DATA_FRAME_MAX_SIZE = 128;
const uint8_t DATA_MAX_SIZE = DATA_FRAME_MAX_SIZE - 4;  // exclude crc
const uint8_t DATA_MIN_SIZE = 2;

const uint8_t DATA_OFFSET_FROM_START = 4;

const uint8_t DATA_FRAME_SOURCE = 0;
const uint8_t DATA_FRAME_DEST = 1;
const uint8_t DATA_FRAME_OPCODE1 = 2;
const uint8_t DATA_FRAME_DATA_LENGTH = 3;
const uint8_t DATA_FRAME_READWRITE_FLAGS = 5;
const uint8_t DATA_FRAME_OPCODE2 = 2;

// Sensor addresses for central unit

const uint8_t SENSOR_ADDRESS_OUT_TEMP = 0x02;   // Outdoor temperature sensor
const uint8_t SENSOR_ADDRESS_IN_TEMP = 0x03;    // Indoor temperature sensor

// adds all the sensors configured in yaml to the component



struct DataFrame {
  union {
    uint8_t raw[DATA_FRAME_MAX_SIZE];
    struct {
      uint8_t source;
      uint8_t dest;
      uint8_t opcode1;
      uint8_t data_length;
      uint8_t data[DATA_MAX_SIZE];
    };
  };

  /**
   * Get size of the raw data (needs data_length set).
   * Returns 0 if not present.
   */
  size_t size() const {
    if (!validate_bounds())
      return 0;

    return DATA_OFFSET_FROM_START + data_length + 1;  // 1 for CRC byte end the end
  }

  /**
   * Check if data frame looks good size wise
   */
  bool validate_bounds() const { return data_length >= DATA_MIN_SIZE && data_length <= DATA_MAX_SIZE; }

  /**
   * Check if CRC in data matches calcualted CRC.
   */
  bool validate_crc() const {
    if (!validate_bounds())
      return false;

    return crc() == calculate_crc();
  }

  /**
   * Get CRC byte at the end of the data.
   * Returns 0 if not yet available
   */
  uint8_t crc() const {
    if (!validate_bounds())
      return 0;

    return raw[size() - 1];
  }

  /**
   * Calculates CRC on the current data by creating an XOR sum
   */
  uint8_t calculate_crc() const {
    if (!validate_bounds())
      return 0;

    uint8_t result = 0;
    size_t len = size() - 1;  // exclude CRC byte and the end
    for (size_t i = 0; i < len; i++) {
      result ^= raw[i];
    }
    return result;
  }

  void reset() {
    for (size_t i = 0; i < DATA_FRAME_MAX_SIZE; i++) {
      raw[i] = 0;
    }
  }

  std::vector<uint8_t> get_data() const { return std::vector<uint8_t>(raw, raw + size()); }
};

struct DataFrameReader {
  // Reads a data frame byte by byte, accumulating bytes until a complete frame is received
  // Returns true when a complete frame is received, false otherwise
  // The frame is stored in the `frame` member variable
  DataFrame frame;
   // Count of "Ignoring noise" events (external counter owned by the logger)
  uint32_t *noise_counter_ptr = nullptr;

  inline void set_noise_counter(uint32_t *p) { this->noise_counter_ptr = p; }
  bool crc_valid{false};
  bool complete{false};
  uint8_t  data_index_{0};
  uint16_t expected_total_{0};  // header(4) + payload(len) + crc(1)

  void reset() {
    frame.reset();
    crc_valid       = false;
    complete        = false;
    data_index_     = 0;
    expected_total_ = 0;
  }

  bool put(uint8_t byte) {
    // Ignore a common noise byte at start
    if (data_index_ == 0 && (byte != 0x00 && byte != 0x04 &&byte != 0x01 && byte != 0x40))  {
      ESP_LOGV("READER", "Ignoring noise");
      if (noise_counter_ptr) (*noise_counter_ptr)++;  // <-- bump "noise" metric
      return false;
    }
    // Store byte
    frame.raw[data_index_] = byte;

    // When the length byte arrives (raw[3]), compute the expected total size
    if (data_index_ == 3) {
      frame.data_length = frame.raw[3];  // keep DataFrame fields in sync
      if (!frame.validate_bounds()) {    // early length sanity check
        ESP_LOGV("READER", "Invalid length 0x%02X; resetting reader", frame.data_length);
        // Resync
        reset();
        return false;
      }
      expected_total_ = DATA_OFFSET_FROM_START + frame.data_length + 1;  // 4 + len + crc
    }

    data_index_++;

    // If we know how many bytes we expect, finish only when we have them all
    if (expected_total_ > 0 && data_index_ >= expected_total_) {
      // We have the whole frame (header + payload + CRC)
      crc_valid = frame.validate_crc();
      complete  = true;

      // Prepare reader for the next frame
      data_index_     = 0;
      expected_total_ = 0;

      return true;
    }

    // Guard against runaway input
    if (data_index_ == DATA_FRAME_MAX_SIZE) {
      ESP_LOGW("READER", "Went over buffer; resetting");
      reset();
    }

    return false;
  }

private:
};


class ToshibaAbLogger : public Component, public uart::UARTDevice {

 public:

  ToshibaAbLogger() = default;
  void dump_config() override;
  void setup() override;
  void loop() override;

  
  uint8_t master_address_ = 0x00;
  void set_master_address(uint8_t address);

  bool receive_data(std::vector<uint8_t> data);
  bool receive_data_frame(const struct DataFrame *frame);


protected:
  DataFrameReader data_reader;

  void process_received_data(const struct DataFrame *frame);
 

  // callbacks
//  CallbackManager<void(const struct DataFrame *frame)> set_data_received_callback_{};

 private:
// noise measurements
  // -------- Metrics (5-minute sliding window, logged every 20 s) --------
  static constexpr uint32_t METRIC_BUCKET_MS = 20 * 1000;  // 20 s
  static constexpr uint8_t  METRIC_BUCKETS   = 15;         // 15 × 20 s = 5 min
  struct MetricBucket {
    uint32_t failed_crc{0};
    uint32_t noise{0};
  };
  MetricBucket metric_buckets_[METRIC_BUCKETS]{};
  uint8_t      metric_idx_{0};              // points to the "just finished" bucket
  uint32_t     metric_bucket_start_ms_{0};  // start time of the current 20 s bucket
  uint32_t     metric_window_start_ms_{0};  // first time we started metrics
  // live counters for the *current* 20 s bucket
  uint32_t     metric_failed_live_{0};
  uint32_t     metric_noise_live_{0};
  // helper
  void metrics_roll_and_log_();




  uint32_t loops_without_reads_ = 0;
  uint32_t loops_with_reads_ = 0;
  uint32_t last_read_millis_ = 0;
  bool can_read_packet = false;
  uint32_t last_received_frame_millis_ = 0;
  uint32_t last_master_alive_millis_ = 0;


};



}  // namespace toshiba_ab


}  // namespace esphome


