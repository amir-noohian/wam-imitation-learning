# Single-arm state protocol v1

UDP destination defaults to `127.0.0.1:6560`. Every packet is exactly 136 bytes,
packed without padding, little-endian. Doubles are IEEE-754 binary64.

| Offset | Field | Type |
|---:|---|---|
| 0 | magic: ASCII WAM7 | 4 bytes |
| 4 | protocol version: 1 | uint32 |
| 8 | sequence, incremented each sample | uint64 |
| 16 | sender monotonic time in nanoseconds | uint64 |
| 24 | joint positions, J1 through J7, radians | 7 doubles |
| 80 | joint velocities, J1 through J7, rad/s | 7 doubles |

Python struct format: `<4sIQQ14d`.

The sequence resets when the producer restarts. Each recording must use one
producer session. The receiver rejects wrong-sized/versioned/non-finite packets
and can restrict the sender IP. There is no authentication, acknowledgement, retry,
or robot command channel. Use a trusted robot network. Packet loss is recorded;
source timestamps, not receive times or assumed loop rates, determine demo timing.
