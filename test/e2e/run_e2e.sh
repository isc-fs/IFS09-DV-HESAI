#!/usr/bin/env bash
# End-to-end check without a lidar: build, then run the node against fake_atx.py for each
# config variant and print the published cloud layout. Run inside a ROS 2 Humble environment
# from the repo root: test/e2e/run_e2e.sh
set -eo pipefail
REPO=$(cd "$(dirname "$0")/../.." && pwd)
WS=${WS:-/tmp/hesai_e2e_ws}
CORR="$REPO/src/driver/HesaiLidar_SDK_2.0/correction/angle_correction/ATX_Angle_Correction_File_V42.dat"

mkdir -p "$WS/src" && rm -rf "$WS/src/hesai" && cp -r "$REPO" "$WS/src/hesai"
(cd "$WS" && colcon build --event-handlers console_direct- >/dev/null)
source "$WS/install/setup.bash"

variant() {  # name, extra driver yaml, extra ros yaml
  local cfg="$WS/$1.yaml"
  sed -e "s|use_ptc_connected: true|use_ptc_connected: false|" \
      -e "s|correction_file_path: \"Your correction file path\"|correction_file_path: \"$CORR\"|" \
      -e "s|device_ip_address: 192.168.1.201|device_ip_address: \"\"|" \
      -e "s|send_imu_ros: true|send_imu_ros: false|" \
      "$REPO/config/config.yaml" > "$cfg"
  python3 - "$cfg" "$2" "$3" <<'PY'
import sys, yaml
p, drv, ros = sys.argv[1:]
c = yaml.safe_load(open(p))
c['lidar'][0]['driver'].update(yaml.safe_load(drv) or {})
c['lidar'][0]['ros'].update(yaml.safe_load(ros) or {})
yaml.safe_dump(c, open(p, 'w'))
PY
  echo "=== $1"
  ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -p config_path:="$cfg" >"$WS/$1.log" 2>&1 &
  local node=$!
  sleep 2
  python3 "$REPO/test/e2e/fake_atx.py" --frames 40 &
  local fake=$!
  python3 "$REPO/test/e2e/check_cloud.py" || { tail -20 "$WS/$1.log"; }
  kill $fake $node 2>/dev/null || true
  wait $node 2>/dev/null || true
}

variant hesai_default "" ""
variant isc "" "{point_format: isc}"
variant isc_drop_last "" "{point_format: isc, drop_zero_points: true, header_stamp: last_packet}"
variant isc_remake_ring "{remake: {enable: true, use_ring: true, max_elev_scan: 116}}" "{point_format: isc, drop_zero_points: true}"
