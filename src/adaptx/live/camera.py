"""PNG encoding of the ego camera's latest frame, for display only.

Pure Python plus ``zlib`` and NumPy - no imaging dependency. The camera is a
visualisation aid: nothing in perception reads it, and this module never
sees anything but pixels.
"""

from __future__ import annotations

import struct
import zlib

import numpy as np


def _chunk(kind: bytes, payload: bytes) -> bytes:
    crc = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)


def encode_png_from_bgra(bgra: bytes, width: int, height: int) -> bytes:
    """Encode a BGRA byte buffer (CARLA's camera layout) as an RGB PNG."""
    expected = width * height * 4
    if len(bgra) != expected:
        raise ValueError(f"expected {expected} bytes for {width}x{height} BGRA, got {len(bgra)}")
    pixels = np.frombuffer(bgra, dtype=np.uint8).reshape(height, width, 4)
    rgb = pixels[:, :, [2, 1, 0]]
    rows = np.concatenate([np.zeros((height, 1), dtype=np.uint8), rgb.reshape(height, -1)], axis=1)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return b"".join(
        [
            b"\x89PNG\r\n\x1a\n",
            _chunk(b"IHDR", header),
            _chunk(b"IDAT", zlib.compress(rows.tobytes(), 3)),
            _chunk(b"IEND", b""),
        ]
    )


__all__ = ["encode_png_from_bgra"]
