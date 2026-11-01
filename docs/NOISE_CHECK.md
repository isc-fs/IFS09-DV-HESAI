# Is the ATX noise caused by the driver?

Check done 2026-10-31 against the noise audit of bag `manual_20260920_154527`
(*IFS09 DV ATX LiDAR Noise Audit v1.0*, 22 September 2026), the driver code
([DRIVER_INTERNALS.md](DRIVER_INTERNALS.md)) and Hesai's public issue trackers.

## Short answer

No. Nothing in the audit is caused by the driver's decoding. Two findings come from how
the cloud is *delivered* (message size and publish thread), one is driver design (no-return
points kept), and one is a sensor behaviour on the lowest channels that the audit had not
identified. Everything else is physics or pipeline code.

## Finding by finding

| Audit finding | Driver? | Evidence | Known issue? | What to do |
|---|---|---|---|---|
| Range noise within spec to 12 m | No | Sensor property | — | Nothing |
| Far ground noisy and flickering (beyond 12 m, intensity 2–8) | No | Grazing incidence and weak returns: σ follows intensity, not channel or face | Expected physics | Possibly gate on the confidence byte later (roadmap #13) |
| 11.9 % of clouds missing from the bag | **Delivery, not decoding** | 0 of 1.52 M UDP packets lost in the driver; whole 4.5 MB clouds lost after it | **Yes, twice**: ROS 2 docs, *DDS tuning*: Cyclone DDS drops large messages with default socket buffers. Hesai [ROS_2.0#91](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0/issues/91): `publish()` runs in the decode thread, so extra subscribers cause loss; fix promised for a later version, not on master | Cyclone tuning on the DVPC (roadmap #15, implemented in [IFS09-DV-PIPELINE#23](https://github.com/isc-fs/IFS09-DV-PIPELINE/pull/23), not installed); smaller clouds with `point_format: isc` + `drop_zero_points`; at most one heavy subscriber, or a publish thread (roadmap #14) |
| No-return points as (0,0,0), 8–14 % | **Yes, by design** | Upstream keeps every slot; see DRIVER_INTERNALS | — | `drop_zero_points: true` (this fork) |
| Even and odd clouds use different index maps | Yes, by design | Frame = one mirror face; corrections have even and odd azimuth tables | — | Split per-beam statistics by face, or use the ring grid (`remake`) |
| ~3–17 returns per cloud more than 20 cm below the ground | **Sensor, lowest channels** | All of them come from rings 60–63 and 112–115 (see below) | Not found in Hesai's trackers | Mask those 8 channels (roadmap #16) |
| "Dead" channels 62, 114, 115 | **Sensor, lowest channels** | Not dead: see below | Not found | Same as above; ask Hesai (roadmap #17) |
| Lidar pitched 8.5–8.9°, TF says 0° | No | Mounting / TF | — | Fix the TF in the pipeline |
| Cone template fit ignores ground offset; residual gate | No | Pipeline `cone_detection` | — | Pipeline fix |
| Per-point time is per packet | Yes, by design | ~117 µs steps | **Yes**: Hesai [SDK_2.0#72](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/issues/72), open | Good enough for deskew; no action |

## Ruled out: wrong angle correction

Hesai [SDK_2.0#65](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/issues/65) reports an
ATX cloud with "large noise" caused by a wrong angle-correction file. The bundled ATX file has
100 channels while ours has 116, so a fallback to it would leave rings 100–115 at 0° elevation.
In the bag, every one of the 116 rings has its own elevation (+5.84° to −12.80°), and the even
and odd faces agree on each ring's elevation to within 0.005°. The correction comes from the
lidar and is consistent.

## The lowest channels have a ~4 m near-range floor

Rings 60–63 and 112–115 are the four lowest beams of each of the two laser banks (−9.5° to
−12.8°).

| Ring | Elevation | Returns | Range of returns (p5 / p50 / p95) |
|---|---|---|---|
| 59 (for comparison) | −9.1° | 98 % | 3.35 / 3.57 / 4.28 m |
| 60 | −9.8° | 6.7 % | 4.01 / 4.14 / 4.46 m |
| 61 | −10.7° | 0.7 % | 4.01 / 4.07 / 4.25 m |
| 62 | −11.7° | ~0 % (17 points) | 4.00 / 4.03 / 4.07 m |
| 63 | −12.8° | 0.6 % | 4.00 / 4.04 / 4.12 m |
| 112 | −9.5° | 6.5 % | 4.09 / 4.23 / 4.55 m |
| 113 | −10.3° | 0.6 % | 4.07 / 4.18 / 4.37 m |
| 114 | −11.2° | ~0 % (5 points) | 4.10 / 4.15 / 4.16 m |
| 115 | ? | 0 % | — |

- No return from these channels is ever closer than 4.0 m, while ring 59, just above them,
  sees the ground from 3.35 m.
- With the sensor pitched ~8.9° nose-down, their ground hit is closer than 4 m in the middle
  of the FOV and further only towards the sides. They return only where it is beyond 4 m.
- The returns they do give sit on the ground (median −3 cm), but the tail is far below it
  (5th percentile −45 cm). In the parked phase **all** below-ground returns (17.2 per cloud)
  come from these 8 channels.

The driver has no range limit in the ATX path, so the floor is applied in the sensor.
It is not listed in Hesai's public issues. Whether it is a fixed design behaviour or a
setting is unknown; ask Hesai.

Practical effect: those 8 channels contribute almost no useful data on our mounting and are
the only source of below-ground strays. Masking them costs ~0.6 % of returns.

## Not verified

- Whether the firetime correction was loaded from the lidar (start-up log; roadmap #3). If it
  was not, the error would be a fixed per-channel azimuth offset (up to 59 µs × mirror speed),
  identical every frame. That is not random noise, so it cannot explain the audit's
  repeatability or flicker figures.
- Whether the DVPC already has Cyclone buffer tuning: nothing in the pipeline's versioned
  deploy scripts sets it.
