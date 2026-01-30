"""
Format raw sample buffer into input dict for CMJ (and other jump) analysers.
"""

from typing import List, Dict, Any

try:
    import numpy as np
except ImportError:
    np = None


def buffer_to_cmj_input(
    samples: List[List[int]],
    sample_rate_hz: float = 1000.0,
) -> Dict[str, Any]:
    """
    Convert a list of 8-channel samples to the format expected by CMJ_Analyser.compute_cmj_test().

    Args:
        samples: List of samples; each sample is [L1, L2, L3, L4, R5, R6, R7, R8] (8 ints, 10g units).
        sample_rate_hz: Sample rate in Hz for building time axis.

    Returns:
        {"left_force": array, "right_force": array, "total_force": array, "index": array}
        "index" is time in seconds. Forces are in raw 10g units.
    """
    if not samples:
        return {
            "left_force": [],
            "right_force": [],
            "total_force": [],
            "index": [],
        }

    n = len(samples)
    left_force = [sum(s[0:4]) for s in samples]
    right_force = [sum(s[4:8]) for s in samples]
    total_force = [l + r for l, r in zip(left_force, right_force)]
    index = [i / sample_rate_hz for i in range(n)]

    if np is not None:
        return {
            "left_force": np.array(left_force, dtype=float),
            "right_force": np.array(right_force, dtype=float),
            "total_force": np.array(total_force, dtype=float),
            "index": np.array(index, dtype=float),
        }
    return {
        "left_force": left_force,
        "right_force": right_force,
        "total_force": total_force,
        "index": index,
    }
