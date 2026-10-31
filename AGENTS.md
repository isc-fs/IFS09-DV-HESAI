# IFS09-DV-HESAI

ISC Formula Student fork of Hesai's ROS 2 driver, used for the IFS09 Hesai ATX lidar.

## Read first

- `docs/ISC.md`: what this fork adds and how to configure it.
- `docs/DRIVER_INTERNALS.md`: how the ATX is decoded; read before touching the decoder.
- `docs/ROADMAP.md`: backlog, decisions, what is blocked on the car.

## Rules

- Every new option must default to the behaviour the car runs today. Upstream code stays
  as-is unless a change is needed; keep ISC code in `src/manager/isc_*` where possible.
- Do not modify the SDK submodule (`src/driver/HesaiLidar_SDK_2.0`) unless unavoidable; if you
  must, record why in `docs/ROADMAP.md`.
- Branches: `feat/N-…` / `fix/N-…`; PRs target `dev`, never `master` (`master` mirrors upstream).
- Pipeline consumers must read PointCloud2 fields by name and offset, never by assumed stride.

## Build and test (ROS 2 Humble, Ubuntu 22.04)

```bash
rosdep install --from-paths . --ignore-src -y
colcon build && colcon test --packages-select hesai_ros_driver && colcon test-result --verbose
test/e2e/run_e2e.sh   # lidar-free end-to-end run with synthetic ATX packets
```

On the macOS dev machine, run these through the `ros2-humble` Docker launcher.
