"""Run with the Python environment containing ESPHome."""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BASE = f"""esphome:
  name: uart-receiver-test
esp8266:
  board: esp12e
logger:
  baud_rate: 0
external_components:
  - source:
      type: local
      path: {ROOT / 'components'}
    components: [toshiba_ab]
toshiba_ab:
  rx_pin: 13
  baud_rate: 2400
  parity: EVEN
  frame_gap: 20ms
"""
cases = [
    ('owned-rx', BASE, True),
    ('unswapped-rx', BASE.replace('rx_pin: 13', 'rx_pin: 3'), True),
    ('normal-mode', BASE + '  debug_mode: normal\n', True),
    ('no-parity-two-stop', BASE.replace('parity: EVEN', 'parity: NONE\n  stop_bits: 2'), True),
    ('unsupported-rx', BASE.replace('rx_pin: 13', 'rx_pin: 12'), False),
    ('logger-conflict', BASE.replace('baud_rate: 0', 'baud_rate: 115200'), False),
    ('uart-conflict', BASE + 'uart:\n  rx_pin: GPIO3\n  baud_rate: 2400\n', False),
    ('zero-gap', BASE.replace('20ms', '0ms'), False),
    ('unrepresentable-gap', BASE.replace('20ms', '1s'), False),
    ('unsupported-platform', BASE.replace('esp8266:\n  board: esp12e', 'esp32:\n  board: esp32dev'), False),
]
with tempfile.TemporaryDirectory() as temp:
    for name, config, valid in cases:
        path = Path(temp) / f'{name}.yaml'
        path.write_text(config)
        proc = subprocess.run([sys.executable, '-m', 'esphome', 'config', str(path)], capture_output=True, text=True)
        assert (proc.returncode == 0) == valid, (name, proc.stdout, proc.stderr)
        print('PASS:', name)
