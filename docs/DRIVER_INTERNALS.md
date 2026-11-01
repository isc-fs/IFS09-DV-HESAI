# How the driver handles the ATX

Reference notes from reading the code (upstream v2.0.12, SDK commit `357ff14`) and checking it
against the car bag `manual_20260920_154527` (2026-09-20). Read this before changing the
decoder. Terms are explained in [GLOSSARY.md](GLOSSARY.md).

## Data flow

```
UDP :2368 ──> receive thread ──> packet ring buffer ──> parse thread
                                                      │  DecodePacket: copy the packet, check the frame bit
                                                      │  ComputeXYZI:  packet -> points (4 worker threads)
                                                      ▼
                                       frame complete ──> point-cloud callback ──> ToRosMsg ──> /lidar_points
```

- The point-cloud callback runs **on the parse thread**. Slow conversion or a blocking publish
  delays decoding of the next frame.
- Corrections (angle and firetime) are fetched from the lidar over its configuration TCP
  connection (PTC, port 9347) at start-up. The files under `correction/` are only a fallback.

## Files that matter

| File | What it does |
|---|---|
| `src/driver/HesaiLidar_SDK_2.0/libhesai/UdpProtocol/udp_protocol_v4_7.h` | ATX packet structs and correction-file formats |
| `src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/include/udp4_7_parser.h` | ATX decoding: `DecodePacket`, `ComputeXYZI` |
| `src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/include/general_parser.h` | Shared helpers: FOV filters, transform, remake grid, packet-loss counter |
| `src/driver/HesaiLidar_SDK_2.0/driver/hesai_lidar_sdk.hpp` | Frame assembly, compaction, callbacks |
| `src/manager/source_drive_common.hpp` | YAML → driver parameters |
| `src/manager/source_driver_ros2.hpp` | ROS 2 node: publishers, message conversion |
| `src/manager/isc_cloud.hpp` | ISC point format (this fork) |

## ATX packet (UDP protocol 4.7)

- 116 channels (lasers) per block, 2 blocks per packet, 232 points per packet.
- Per point: distance (uint16 × 4 mm on our unit), reflectivity (uint8), confidence (uint8).
- Per block: azimuth in 1/256°. The "fine azimuth" byte exists but the ATX parser ignores it.
- Tail: frame bit, motor speed, return mode, microsecond timestamp, UTC seconds.
- Packet length on the car: about 1 kB.

## Frames

- A frame ends when the **frame bit in the packet tail flips**. That bit tracks which face of
  the scanning mirror is in use, so consecutive frames come from alternating mirror faces.
- The angle corrections have separate azimuth tables for even and odd faces. This is why
  even and odd frames use different index→direction maps (seen in the noise audit). It is
  physical, not a bug.
- Car: 750 packets per frame → 750 × 232 = 174 000 points, 10 Hz.
- Points with no return are kept as (0,0,0); about 8.5 % of the cloud. Upstream never removes
  them. The ISC option `drop_zero_points` does.

## Turning a packet into points (`ComputeXYZI`)

For each channel of each block:

1. `azimuth = block azimuth / 256`
2. Add the firetime correction (`firetime × motor speed`, sign depends on the mirror face).
3. Add the per-channel azimuth correction (even or odd table).
4. `elevation = per-channel elevation` (+ an azimuth-dependent adjustment for correction-file version 3).
5. Skip the point if a FOV filter excludes it.
6. `x = d·cos(el)·sin(az)`, `y = d·cos(el)·cos(az)`, `z = d·sin(el)`. Azimuth 90° is +x
   (forward), so the frame is REP-103-compatible.
7. Optional rigid transform (`transform_flag`), then optional remake grid placement.

No optical-centre (distance) correction is applied for the ATX, even with
`distance_correction_flag: true`.

## Timestamps

- `use_timestamp_type: 0`: lidar time from the packet tail (PTP-synced on the car).
  `1`: host receive time.
- Every point in a packet gets the **packet** time, not a per-laser time. On the car that is
  750 distinct times per frame, ~117 µs apart, spanning ~89.8 ms.
- Upstream header stamp = first packet. ISC option `header_stamp` can use the last packet.
- On the car, the bag shows `/lidar_points` logged ~110 ms after its stamp: ~90 ms of frame
  assembly plus ~20 ms of conversion and delivery.

## What upstream drops or hides

| Item | Upstream | This fork |
|---|---|---|
| Per-point confidence | Decoded, then lost (point type has no field) | Kept in `point_format: isc` |
| Intensity | uint8 widened to float32 | uint8 in `isc` |
| Remake (organized grid) | In the SDK, not in the ROS config | `remake` block |
| PTP status topic | Supported, commented out | Recommended in [ISC.md](ISC.md) |
| Publisher QoS | RELIABLE, depth 10 | `point_cloud_qos`, default best effort |

## SDK quirks

- Remake: any negative `min_azi` / `min_elev` is replaced by the SDK default, so negative
  values cannot be set.
- Remake defaults are written into the frame's copy of the parameters at decode time, not into
  the parameters the ROS node holds. Read grid sizes from the frame (`frame.fParam`).
- The bundled `ATX_Angle_Correction_File_V42.dat` has 100 channels; our unit has 116. The car
  uses the corrections from the lidar, so this only matters for offline decoding without PTC.

## Bag facts (2026-09-20)

| Item | Value |
|---|---|
| Points per frame | 174 000 (750 packets × 232) |
| Point layout | packed 26 bytes: x, y, z, intensity (f32), ring (u16), timestamp (f64) |
| No-return points | 8.5 % |
| Distinct timestamps per frame | 750, ~117 µs apart |
| Stamp → bag log time | ~110 ms |
| Intensity values | whole numbers 0–255 |
| Raw packets recorded | no (`/lidar_packets` absent) |
