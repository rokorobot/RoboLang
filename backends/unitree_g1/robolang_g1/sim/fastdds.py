"""Publishing a DDS sample at the physics loop's rate without serializing it in Python (phase 4 prerequisite).

`rt/lowstate` (250 Hz) and the Dex3 hand states (100 Hz each) are written from the 2 ms physics loop. The SDK's `Write`
serializes the sample with CycloneDDS-Python's pure-Python CDR machinery: measured 247 us for one `LowState_` (37
recursive `serialize` calls a write), against 11 us to fill it. At 250 Hz that is 6% of a core, and more than half of
everything the loop does in a step (docs/evidence/g1-phase4-prereqs.md). On the 4-vCPU CI runner the loop then spends
so long in `Write` that a descheduled step costs tens of milliseconds, and the margin to a 5% slow second is small.

A `LowState_` has a fixed layout: arrays of fixed length, no sequences. So it is serialized **once**, the byte offsets
of the fields that change (joint position, velocity and torque, the IMU, `tick`) are found by giving each a unique
value and looking for it in the bytes, and from then on a publish writes the new values into that buffer with numpy
and hands the bytes to `DataWriter.write`, whose `isinstance` check the sample still passes (it is the SDK's own
message object) and whose `serialize()` it answers with the buffer. What goes on the wire is **the same bytes** the SDK
would have written (a test compares them for random values), so every subscriber, the SDK's own included, reads what
it always did.
"""

from __future__ import annotations

import struct
from collections.abc import Callable, Sequence

import numpy as np

FLOAT_SENTINEL = 1.0e6  # + k: exactly a float32 (spacing 0.0625 there), so its bytes can be searched for
INT_SENTINEL = 0x70000000


class PatchedSample:
    """A message object whose `serialize()` returns a byte buffer in which named float32 / uint32 fields are
    overwritten in place. `msg` is the SDK message (constants already set); `floats` maps a group name to one setter
    per element (`lambda v: setattr(motor, "q", v)`), `ints` likewise for uint32 fields. The setters are only used to
    locate the fields: from then on `set` and `set_int` write the buffer and leave the message's own fields alone."""

    def __init__(
        self,
        msg,
        floats: dict[str, Sequence[Callable[[float], None]]],
        ints: dict[str, Sequence[Callable[[int], None]]] | None = None,
    ):
        patterns: dict[tuple[str, int], tuple[bytes, bool]] = {}
        k = 0
        for name, setters in floats.items():
            for i, setter in enumerate(setters):
                setter(FLOAT_SENTINEL + k)
                patterns[(name, i)] = (struct.pack("<f", FLOAT_SENTINEL + k), False)
                k += 1
        for name, setters in (ints or {}).items():
            for i, setter in enumerate(setters):
                setter(INT_SENTINEL + k)
                patterns[(name, i)] = (struct.pack("<I", INT_SENTINEL + k), True)
                k += 1
        template = bytes(msg.serialize())
        self.buf = bytearray(template.ljust((len(template) + 3) & ~3, b"\0"))
        self._f4 = np.frombuffer(self.buf, dtype="<f4")
        self._u4 = np.frombuffer(self.buf, dtype="<u4")
        self._index: dict[str, np.ndarray] = {}
        for name in [*floats, *(ints or {})]:
            n = len(floats.get(name) or (ints or {})[name])
            offsets = []
            for i in range(n):
                pattern, _ = patterns[(name, i)]
                at = template.find(pattern)
                if at < 0 or at % 4 or template.find(pattern, at + 1) >= 0:
                    raise ValueError(f"field {name}[{i}] is not a unique, aligned value in the serialized sample")
                offsets.append(at // 4)
            self._index[name] = np.array(offsets)
        msg.serialize = self._serialize  # the instance answers for itself: the SDK's write() asks it for the bytes
        self.msg = msg

    def _serialize(self, *args, **kwargs) -> bytes:
        return bytes(self.buf)

    def set(self, name: str, values) -> None:
        self._f4[self._index[name]] = values

    def set_int(self, name: str, values) -> None:
        self._u4[self._index[name]] = values
