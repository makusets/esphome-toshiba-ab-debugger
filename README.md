# Toshiba AB traffic debugger

An ESPHome component for inspecting Toshiba AB and unknown serial traffic on an
ESP8266. It collects raw bytes and logs a frame when the line goes silent.
ESPHome handles Wi-Fi, OTA, and logging; the component owns hardware UART0.

## Setup

Add this to your existing ESPHome device configuration:

```yaml
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
  parity: AUTO
  frame_gap: 20ms
```

Remove any previous `uart:` section. RX uses GPIO13 through UART0 pin swapping;
GPIO3 is also supported. This component only receives and holds GPIO16 low for
boards that use it as the transceiver enable. Use an appropriate bus interface;
hardware information is in the [main repository](https://github.com/makusets/esphome-toshiba-ab/tree/main/hardware).

See [example.yaml](example.yaml) for a minimal local configuration. Keep your
existing Wi-Fi, API, and OTA settings to view logs and update the device remotely.

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
and `parity` (`AUTO`, `NONE`, `EVEN`, or `ODD`, default `AUTO`). `debug_mode: normal` enables the original
Toshiba length/CRC parser; `master` selects its master address.

## Automatic parity discovery

With `parity: AUTO`, the receiver tests EVEN, ODD, then NONE. **Baud rate stays
fixed** at `baud_rate` (default 2400); data bits and stop bits stay fixed too.
Explicit EVEN, ODD, or NONE skips discovery.

Allow about **60 seconds** with a frame at least every five seconds: each trial
lasts at least 20 seconds and needs at least three complete frames and 12 received
bytes. Very short frames, missing traffic, capture loss, or a busy line extend
this time. Silence is never evidence of a correct setting. Changes wait for an
empty capture queue, an empty UART FIFO, and an idle RX line; RX edge interrupts
also postpone a switch when a byte has started but is not yet in the FIFO.

The detector compares the fraction of frames with parity/framing errors and
counts error interrupt events, rather than claiming exact damaged-byte counts.
NONE must have repeated clean captures and both parity-enabled modes must fail;
it cannot win merely because parity checking is disabled. If settings remain
indistinguishable, discovery finishes as uncertain and keeps EVEN.

Discovery runs **once per boot**. Detected and uncertain results are final;
later errors only generate warnings and never trigger another scan.

A diagnostic text sensor named **UART Settings** is enabled by default. It shows
states such as `Discovering: 2400 baud, 8O1`, `Detected: 2400 baud, 8E1`,
`Uncertain: 2400 baud, 8E1`, or `Configured: 2400 baud, 8E1` for manual
parity. Insufficient samples produce `Waiting for traffic`. It is exposed through
your existing ESPHome API/MQTT configuration. To rename it or turn it off:

```yaml
toshiba_ab:
  diagnostic:
    name: Serial Discovery
  # Alternatively: diagnostic: false
```

[Test instructions](tests/README.md) cover capture behavior and configuration.
