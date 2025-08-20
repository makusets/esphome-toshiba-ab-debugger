#include "toshiba_ab.h"


namespace esphome {
namespace toshiba_ab {



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
  ESP_LOGV(TAG, "%s%s", prefix.c_str(), res.c_str());
}


void ToshibaAbLogger::dump_config() {
  ESP_LOGCONFIG(TAG, "Toshiba Protocol sniffer");
}

void ToshibaAbLogger::setup() {
  ESP_LOGD(TAG, "Setting up Toshiba Protocol sniffer...");
  this->can_read_packet = true;  // start consuming immediately
  last_master_alive_millis_ = millis();  // reset last alive time
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

    return false;
  }
  process_received_data(frame);
  return true;
}


void ToshibaAbLogger::loop() {

  uint8_t bytes_read = 0;

  while (available()) {
    int byte = read();
    if (byte >= 0) {
      bytes_read++;

      if (!can_read_packet)
        continue;  // wait until can read packet

      if (data_reader.put(byte)) {
        // packet complete

        last_received_frame_millis_ = millis();

        auto frame = data_reader.frame;

        if (!receive_data_frame(&frame)) {
        }

        data_reader.reset();

        // read next packet (if any in the next loop)
        // the smallest packet (ALIVE) is 32ms wide,
        // which means there are max ~31 packets per second.
        // and the loop runs 33-50 times per second.
        // so should be enough throughput to process packets.
        // this ensure that each packet is interpreted separately
        break;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read data");
    }
  }

  if (bytes_read > 0) {
    loops_with_reads_++;
    loops_without_reads_ = 0;

    // ESP_LOGV(TAG, "Bytes of data read: %d", bytes_read);
    // if (!data_reader.complete) {
    //   log_data_frame("Pending", data_reader.frame);
    // }

    last_read_millis_ = millis();
  } else {
    loops_without_reads_++;
    loops_with_reads_ = 0;

    if (last_read_millis_ > 0) {
      auto millis_since_last_read = millis() - last_read_millis_;
      if (millis_since_last_read >= PACKET_MIN_WAIT_MILLIS) {
        // can start reading packet

        if (!data_reader.complete && data_reader.data_index_ > 0) {
          // ESP_LOGW(TAG, "Reset pending frame buffer (%d)",
          // data_reader.data_index_); log_raw_data("Pending: ",
          // data_reader.frame.raw, data_reader.data_index_);
        }
        can_read_packet = true;
        data_reader.reset();
        last_read_millis_ = 0;
      }
    }
  }

  if (last_master_alive_millis_ > 0 && (millis() - last_master_alive_millis_) > LAST_ALIVE_TIMEOUT_MILLIS) {
  ESP_LOGW(TAG, "No master frames for a while (disconnected?)");
  }
}



}  // namespace toshiba_ab
}  // namespace esphome

void esphome::toshiba_ab::ToshibaAbLogger::set_master_address(uint8_t address) {
  this->master_address_ = address;
}
