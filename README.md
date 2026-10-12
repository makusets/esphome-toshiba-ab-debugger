# Toshiba AB traffic debugger

An ESPHome component for inspecting Toshiba AB and unknown serial traffic on an
ESP8266. It collects raw bytes and logs a frame when the line goes silent.
ESPHome handles Wi-Fi, OTA, and logging; the component owns hardware UART0.

## Setup

ESPHome device configuration:

```yaml
esphome:
  name: toshiba-debugger
  friendly_name: toshiba_debugger

esp8266:
  board: esp12e


# Enable Home Assistant API
api:


ota:
  - platform: esphome

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  output_power: 10dB            #lower transmission power, reduces noise on AB line


logger:
  baud_rate: 0
  level: DEBUG
  tx_buffer_size: 1024

external_components:
  - source: github://makusets/esphome-toshiba-ab-debugger
    components: [toshiba_ab]
    refresh: 0s

toshiba_ab:
  rx_pin: 13
  baud_rate: 2400
  parity: EVEN
  frame_gap: 20ms
```

Remove any previous `uart:` section. RX uses GPIO13 through UART0 pin swapping;
hardware information is in the [main repository](https://github.com/makusets/esphome-toshiba-ab/tree/main/hardware).


## Frames and timing

The default `debug_mode: raw` logs every byte in hexadecimal, including short
frames and `FF`/`FE`. Frame boundaries are captured by a UART silence interrupt
and queued before ESPHome logs them.

`frame_gap` should be longer than pauses within a frame and shorter than gaps
between frames. Hardware rounds it up to a whole character time: at 2400 baud,
8 data bits, even parity, and one stop bit, `20ms` becomes approximately **22.9 ms**.
Startup logs show the effective timeout. The hardware supports 1–127 character times.

Frame log messages contain only the hexadecimal bytes, for example:

```text
01:52:11:04:80:86:A1:05:E4
```

UART errors and lost captures are reported separately as readable warnings.
Frames over 128 bytes are logged in fragments with a warning. The receiver still
captures boundaries before logging; long interrupt delays can hide traffic gaps.

Optional settings are `data_bits` (5–8, default 8), `stop_bits` (1 or 2, default 1),
and `parity` (`NONE`, `EVEN`, or `ODD`).

[Test instructions](tests/README.md) cover capture behavior and configuration.
