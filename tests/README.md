Run these commands from the repository root:

```sh
g++ -std=c++17 -Wall -Wextra -DUSE_ESP8266 -Itests/include tests/raw_reader_test.cpp -o /tmp/raw_reader_test
/tmp/raw_reader_test
g++ -std=c++17 -Wall -Wextra tests/parity_detector_test.cpp -o /tmp/parity_detector_test
/tmp/parity_detector_test
g++ -std=c++17 -Wall -Wextra -DUSE_ESP8266 -Itests/include tests/parity_receiver_test.cpp -o /tmp/parity_receiver_test
/tmp/parity_receiver_test
g++ -std=c++17 -Wall -Wextra tests/baud_estimator_test.cpp -o /tmp/baud_estimator_test
/tmp/baud_estimator_test
g++ -std=c++17 -Wall -Wextra -DUSE_ESP8266 -Itests/include tests/baud_receiver_test.cpp -o /tmp/baud_receiver_test
/tmp/baud_receiver_test
python tests/validate_config.py  # Use an environment with ESPHome installed.
```

The C++ tests compile the actual receiver and capture queue with simulated
ESP8266 registers, FIFO, clock, and interrupts. They check hardware UART setup,
silence IRQ boundaries, delayed logging, FF/FE and single-byte captures, retaining
a FIFO tail byte, 128-byte fragmentation, UART errors, queue loss, timestamp
wrap, queue wrap, and shutdown. Logging is forbidden while the simulated ISR runs.

Configuration tests cover native and swapped RX, serial logger conflicts,
competing UART hubs, framing settings, normal mode, hardware timeout limits,
and unsupported pins/platforms. A real firmware build and on-wire validation
remain necessary to verify actual hardware timing and interrupt latency.

Parity tests cover EVEN/ODD/NONE selection, ambiguous and insufficient evidence,
repeated clean NONE frames, capture-loss retries, uptime wrap, fixed default and
custom baud rates, idle-line switching, RX edges, diagnostic state publication,
manual parity, and final results that never restart after later errors.

Baud timing tests cover standard/custom rates, harmonic ambiguity, insufficient
and inconsistent evidence, excluded idle gaps, timestamp/ring wrap, timing-buffer
overflow recovery, suggestions that differ from configured baud, fixed receiver
settings, explicit parity, and final suggestions that remain frozen.
