# ISC fork of the Hesai ROS 2 driver

Fork of [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) for the
IFS09 Hesai ATX. Upstream behaviour is kept unless you set one of the options below; the only
default that differs from upstream is `point_cloud_qos` (see the table).

## Options

All in `config/config.yaml`.

| Key (block) | Values | Default | What it does |
|---|---|---|---|
| `point_format` (`ros`) | `hesai`, `isc` | `hesai` | `hesai` = upstream packed 26-byte XYZIRT. `isc` = 20-byte aligned layout below. |
| `drop_zero_points` (`ros`) | bool | `false` | `isc` only. Drops no-return points, which arrive as (0,0,0) (~8.5 % on the car). Ignored when `remake` is on. |
| `header_stamp` (`ros`) | `first_packet`, `last_packet` | `first_packet` | Which packet time goes in `header.stamp`. A frame spans ~90 ms. |
| `point_cloud_qos` (`ros`) | `sensor_data`, `reliable` | `sensor_data` | Upstream publishes RELIABLE depth 10; the car's previous driver was best effort, which `dv_record.sh` and the pipeline rely on. |
| `remake` (`driver`) | block | off | Organized grid, see below. |

### `isc` point layout

| Field | Type | Offset |
|---|---|---|
| x, y, z | float32 | 0, 4, 8 |
| intensity | uint8 | 12 |
| confidence | uint8 | 13 |
| ring | uint16 | 14 |
| time | float32, seconds relative to `header.stamp` | 16 |

`point_step` is 20 and every field is aligned to its own size. `time` is per UDP packet
(~117 µs steps on the ATX), not per laser firing. `confidence` is the per-point byte from the
packet, which the upstream point type discards.

### `remake` grid

Resamples each frame into a fixed grid so a given index always points the same way, on both
mirror faces. With `use_ring: true` the second axis is the ring, so set `max_elev_scan` to the
channel count (116). The ATX defaults are azimuth 27–153° in 0.1° steps (1260 columns). The
cloud is published with `height` = azimuth bins and `width` = rings (or elevation bins), in
that memory order, and empty slots are (0,0,0) with `time` 0.

`min_azi` / `min_elev` cannot be negative: the SDK treats any negative value as "use the
default".

## Recommended car settings

```yaml
    ros:
      point_format: isc
      drop_zero_points: true
      send_packet_ros: true            # record /lidar_packets: ~1 kB per packet, ~6x smaller than the cloud
      ros_send_ptp_topic: /lidar_ptp   # PTP lock status
```

To re-decode a recorded run with different settings, replay the bag and run the driver with
`source_type: 3` (it subscribes to `ros_recv_packet_topic`).

Consumers must read fields by name and offset, never by assuming a stride.

## Testing

- Unit: `colcon test --packages-select hesai_ros_driver` (`test/test_isc_cloud.cpp`).
- End to end without a lidar: `test/e2e/run_e2e.sh` in a ROS 2 Humble shell. It runs the node
  against `test/e2e/fake_atx.py` (synthetic 4.7 packets) for each option set and prints the
  published layout.

## Before deploying to the DVPC

The DVPC runs an unversioned copy in `~/ros2_ws/src/HesaiLidar_ROS_2.0` with local changes
(at least best-effort QoS). Diff it against this repo and port anything missing before
switching the car over, and copy its `config.yaml` values (IPs, `use_timestamp_type: 0`, FOV).
