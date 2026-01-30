"""
Parse BLE data packets from the force plate.
Packet format: 1 byte sample_count (1-10), then sample_count x 16 bytes (8 x int16 LE: L1..L4, R5..R8).
"""

import struct
from typing import List, Dict, Any


def parse_packet(data: bytes) -> Dict[str, Any]:
    """
    Parse a single BLE notification payload.

    Returns:
        {"sample_count": int, "samples": [[8 ints], ...]}
        Each sample is [L1, L2, L3, L4, R5, R6, R7, R8] in raw 10g units.
    """
    if not data or len(data) < 1:
        return {"sample_count": 0, "samples": []}
    sample_count = data[0]
    if sample_count > 10:
        return {"sample_count": 0, "samples": []}
    expected_len = 1 + sample_count * 16
    if len(data) < expected_len or sample_count == 0:
        return {"sample_count": 0, "samples": []}

    samples: List[List[int]] = []
    for i in range(sample_count):
        off = 1 + i * 16
        local4 = struct.unpack_from("<4h", data, off)
        remote4 = struct.unpack_from("<4h", data, off + 8)
        samples.append(list(local4) + list(remote4))
    return {"sample_count": sample_count, "samples": samples}
