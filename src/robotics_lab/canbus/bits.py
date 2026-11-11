"""Worst-case on-wire length of a classical CAN frame.

The count is 47 control bits (including the 3-bit intermission) plus 8 bits
per data byte, plus the worst-case stuffed bits on the 34 + 8n stuffable bits.
For an 11-bit identifier that is

    47 + 8n + floor((34 + 8n - 1) / 4)

which is 135 bits for an 8-byte frame. The divisor is 4 because a stuff bit
starts a new run of identical bits. This is the estimate used for response-time
analysis (Tindell; corrected form in Davis, Burns, Bril and Lukkien, Real-Time
Systems 35, 2007) and by can-utils ``CFL_WORSTCASE``. It is not a count of the
bits that a particular payload actually stuffed, and it is not CAN FD.
"""

from __future__ import annotations


def frame_bits(dlc: int, extended: bool = False) -> int:
    """Worst-case bits occupied by one classical CAN frame, including intermission."""
    if dlc < 0 or dlc > 8:
        raise ValueError(f"classical CAN DLC must be 0..8, got {dlc}")
    if extended:
        stuffable = 54 + 8 * dlc
        overhead = 54 + 13
    else:
        stuffable = 34 + 8 * dlc
        overhead = 47
    return overhead + 8 * dlc + (stuffable - 1) // 4


def bus_load(frame_bits_list: list[int], times_ns: list[int], bitrate: int) -> tuple[float, float]:
    """Return ``(load_fraction, duration_s)``.

    The window starts at the first frame and ends when the last frame's
    worst-case bits have finished. Load is total worst-case bits divided by
    ``bitrate * duration``.
    """
    if len(frame_bits_list) != len(times_ns):
        raise ValueError("one bit count per timestamp is required")
    if len(times_ns) < 2:
        return 0.0, 0.0
    if bitrate <= 0:
        raise ValueError("bitrate must be positive")
    span_s = (times_ns[-1] - times_ns[0]) / 1e9
    duration_s = span_s + frame_bits_list[-1] / bitrate
    total = sum(frame_bits_list)
    return total / (bitrate * duration_s), duration_s
