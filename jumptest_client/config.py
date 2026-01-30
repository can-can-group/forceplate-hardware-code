"""
Configuration for jumptest_client: BLE UUIDs, sample rate, channel mapping.
"""

# BLE (match gui_ble_client / ESP32 Load Cell BLE Server)
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_CHAR_UUID = "87654321-4321-4321-4321-cba987654321"
COMMAND_CHAR_UUID = "11111111-2222-3333-4444-555555555555"
DEFAULT_DEVICE_NAME = "LoadCell_BLE_Server"

# Sample rate from hardware (~1000 Hz)
SAMPLE_RATE_HZ = 1000

# Channel mapping: each sample = 8 int16 (L1..L4 local, R5..R8 remote)
# left_force = sum(L1..L4), right_force = sum(R5..R8)
CHANNELS_LEFT = (0, 1, 2, 3)   # indices into 8-element sample
CHANNELS_RIGHT = (4, 5, 6, 7)

# Optional: base path for saving/loading raw buffers (offline analysis)
# DATA_DIR = None  # or e.g. pathlib.Path("recordings")
