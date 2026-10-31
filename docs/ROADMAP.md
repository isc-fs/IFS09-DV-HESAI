# Roadmap and backlog

Work on the ATX driver fork, ordered by priority. Sizes: **S** = under a day, **M** = a few
days, **L** = a sprint or more. Terms are in [GLOSSARY.md](GLOSSARY.md).

Last updated 2026-10-31.

## Goal

Get a cloud from the ATX that is smaller, carries everything the sensor measures, and is easy
for the pipeline to use, without ever breaking what runs on the car today.

## Definition of done

A backlog item is done when:

- it builds and `colcon test` passes in ROS 2 Humble;
- `test/e2e/run_e2e.sh` still passes, and the default output is unchanged unless the item
  says otherwise;
- for anything that changes car behaviour: it has run on the car and there is a bag;
- `docs/` is updated.

## Done

| Item | Where |
|---|---|
| Fork upstream as `isc-fs/IFS09-DV-HESAI`, `dev` as default branch | — |
| ISC point format (20-byte aligned, confidence, relative time) | PR #1 |
| Options: `drop_zero_points`, `header_stamp`, `point_cloud_qos` | PR #1 |
| Expose the remake (organized grid) in the YAML | PR #1 |
| Unit tests and lidar-free end-to-end harness | PR #1 |
| Driver internals reference | [DRIVER_INTERNALS.md](DRIVER_INTERNALS.md) |

## Next: needs the DVPC (car computer) reachable

These are blocked until someone is connected to the car (`ssh isc@192.168.2.2` over the
umbilical).

| # | Item | Size | Notes |
|---|---|---|---|
| 1 | **Compare the car's driver copy with this fork** | S | The car runs an unversioned copy in `~/ros2_ws/src/HesaiLidar_ROS_2.0`. It publishes best effort, which upstream never did, so it has local changes. Record its upstream version (`git -C ... log -1`, `git status`, `git diff`), port every local change into this fork, and note them here. |
| 2 | **Bring the car's `config.yaml` into the repo** | S | Save it as `config/ifs09_atx.yaml` (IPs, `use_timestamp_type: 0`, FOV values from the `fov` tool). From then on the config is versioned here. |
| 3 | **Check what the lidar actually sends** | S | From the driver log at start-up: were the angle and firetime corrections loaded from the lidar? Is the confidence status bit set in the packet header? Which return mode (strongest / last / dual)? These also matter for the noise investigation. |
| 4 | **Swap the car over to the fork, defaults only** | M | Build in `~/ros2_ws`, keep the old copy as a rollback. Record a short bag and check that `/lidar_points` is identical in layout, rate and stamps to before. |

## Then

| # | Item | Size | Notes |
|---|---|---|---|
| 5 | **Turn on `point_format: isc` + `drop_zero_points` on the car** | S | `cone_detection` already reads x/y/z by field offset, so it should work unchanged. Check every other `/lidar_points` consumer (recorder tools, Foxglove layouts, analysis scripts) for assumptions about `intensity` being float32 or a `timestamp` field. |
| 6 | **Record raw packets instead of the cloud** | M | `send_packet_ros: true`, then record `/lidar_packets` in the pipeline's `deploy/dv_record.sh` instead of `/lidar_points`. About 6× less data. Write the replay recipe (`source_type: 3`) into the pipeline's `docs/RECORDING.md`. Lets us re-decode old runs with any future driver change. |
| 7 | **Publish PTP status** | S | `ros_send_ptp_topic: /lidar_ptp`; add it to the recorder and to bring-up health checks. |
| 8 | **Choose the header stamp for SLAM** | S | First or last packet; agree with whoever owns SLAM. Point `time` already allows deskewing. |
| 9 | **Turn on CI** | S | Enable GitHub Actions on the fork; build + unit tests + e2e on every PR. |
| 10 | **Upstream sync routine** | S | Pull `HesaiTechnology/HesaiLidar_ROS_2.0` into `master`, merge into `dev`, re-run tests. Do it before each competition. |

## Later

| # | Item | Size | Notes |
|---|---|---|---|
| 11 | **Ring grid + range-image cone detection** | L | Enable `remake` with `use_ring: true`. Ground removal and clustering on a fixed image are O(n) and avoid RANSAC + DBSCAN, which is what limits `cone_detection` throughput. Needs pipeline work. |
| 12 | **Crop in the driver** | M | Range / height / azimuth limits applied in the parse threads, so the pipeline gets fewer points. Needs the real extrinsic (the lidar is pitched ~8.7° down, the TF says 0). Keep one source of truth: either the driver applies the extrinsic and the TF is identity, or neither. |
| 13 | **Use confidence for filtering** | M | Once #3 confirms the byte is filled: study it against the noise audit (far-range flicker) and add a threshold option. |
| 14 | **Faster conversion / publish off the parse thread** | M | Only if #4 shows the ~20 ms conversion + delivery matters. |

## Decision log

| Date | Decision | Why |
|---|---|---|
| 2026-10-31 | Fork rather than patch in place | The car copy had unversioned local changes; we need history and reviews. |
| 2026-10-31 | Keep upstream behaviour as the default for every new option, except best-effort QoS | Swapping the driver must not change what the pipeline sees. Best effort is what the car already runs. |
| 2026-10-31 | Use `time` (float32, relative) for per-point time in the ISC format | Same convention as Velodyne drivers, so deskew tools read it directly; float32 keeps µs resolution relative to the stamp. |
