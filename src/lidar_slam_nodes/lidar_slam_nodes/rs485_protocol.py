"""RS-485 chassis communication protocol encoder/decoder.

Frame format (23 bytes fixed):
  [0xAA][0x55] | [CH1_H][CH1_L] ... [CH10_H][CH10_L] | [0xA5]
  Header(2B)            Data(20B, 10 channels x 2B)          Footer(1B)

Channel range: 1000-2000, center=1500 (neutral/stop).
"""

HEADER = bytes([0xAA, 0x55])
FOOTER = bytes([0xA5])
FRAME_SIZE = 23
NUM_CHANNELS = 10
MIN_VALUE = 1000
MAX_VALUE = 2000
NEUTRAL = 1500


def clamp(value: int) -> int:
    return max(MIN_VALUE, min(MAX_VALUE, value))


def encode_frame(channels: list) -> bytes:
    """Encode 10 channel values into a 23-byte RS-485 frame."""
    if len(channels) != NUM_CHANNELS:
        raise ValueError(f"Expected {NUM_CHANNELS} channels, got {len(channels)}")

    frame = bytearray(HEADER)
    for ch in channels:
        v = clamp(int(ch))
        frame.append((v >> 8) & 0xFF)  # high byte
        frame.append(v & 0xFF)         # low byte
    frame.extend(FOOTER)
    return bytes(frame)


def decode_frame(data: bytes) -> list | None:
    """Decode a 23-byte frame into 10 channel values. Returns None on invalid frame."""
    if len(data) != FRAME_SIZE:
        return None
    if data[0:2] != HEADER or data[-1] != FOOTER[0]:
        return None

    channels = []
    for i in range(NUM_CHANNELS):
        offset = 2 + i * 2
        val = (data[offset] << 8) | data[offset + 1]
        channels.append(clamp(val))
    return channels


def find_frame(data: bytes) -> tuple:
    """Scan byte buffer for a valid frame. Returns (channels, remaining_bytes) or (None, remaining).

    Scans for AA 55 header, then checks if enough bytes exist for a complete frame.
    Returns decoded channels and unconsumed bytes, or None if no valid frame found.
    """
    idx = 0
    while idx <= len(data) - FRAME_SIZE:
        if data[idx] == 0xAA and data[idx + 1] == 0x55:
            candidate = data[idx:idx + FRAME_SIZE]
            result = decode_frame(candidate)
            if result is not None:
                return result, data[idx + FRAME_SIZE:]
            idx += 1
        else:
            idx += 1
    return None, data


def steering_to_channel(angle_rad: float, max_angle: float) -> int:
    """Map steering angle (radians) to channel value [1000, 2000].

    Negative angle = left turn (closer to 1000), positive = right (closer to 2000).
    """
    if max_angle == 0:
        return NEUTRAL
    ratio = max(-1.0, min(1.0, angle_rad / max_angle))
    return clamp(int(NEUTRAL + 500 * ratio))


def channel_to_steering(channel: int, max_angle: float) -> float:
    """Map channel value [1000, 2000] to steering angle (radians)."""
    ratio = (clamp(channel) - NEUTRAL) / 500.0
    return ratio * max_angle


def speed_to_channel(speed: float, max_speed: float) -> int:
    """Map speed (m/s) to channel value [1000, 2000].

    Negative speed = reverse (closer to 1000), positive = forward (closer to 2000).
    """
    if max_speed == 0:
        return NEUTRAL
    ratio = max(-1.0, min(1.0, speed / max_speed))
    return clamp(int(NEUTRAL + 500 * ratio))


def channel_to_speed(channel: int, max_speed: float) -> float:
    """Map channel value [1000, 2000] to speed (m/s)."""
    ratio = (clamp(channel) - NEUTRAL) / 500.0
    return ratio * max_speed
