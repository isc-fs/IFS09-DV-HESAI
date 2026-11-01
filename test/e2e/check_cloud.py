#!/usr/bin/env python3
"""Print the layout of a few /lidar_points messages (best-effort subscriber)."""
import sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

TYPES = {2: 'u1', 4: 'u2', 7: 'f4', 8: 'f8'}


def main():
    rclpy.init()
    node = rclpy.create_node('check_cloud')
    got = []
    node.create_subscription(PointCloud2, '/lidar_points', got.append, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + 15e9
    while len(got) < 4 and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(got) < 4:
        print('FAIL: got', len(got), 'clouds')
        sys.exit(1)
    m = got[-1]
    dt = np.dtype({'names': [f.name for f in m.fields], 'formats': ['<' + TYPES[f.datatype] for f in m.fields],
                   'offsets': [f.offset for f in m.fields], 'itemsize': m.point_step})
    a = np.frombuffer(bytes(m.data), dtype=dt)
    zero = np.mean((a['x'] == 0) & (a['y'] == 0) & (a['z'] == 0))
    t = a['time'] if 'time' in a.dtype.names else a['timestamp'] - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    print(f'height={m.height} width={m.width} point_step={m.point_step} bytes={len(m.data)} dense={m.is_dense}')
    print('fields=' + ','.join(f'{f.name}@{f.offset}' for f in m.fields))
    print(f'zero_frac={zero:.3f} t_rel=[{t.min():.4f},{t.max():.4f}] intensity=[{a["intensity"].min()},{a["intensity"].max()}]'
          + (f' confidence={np.unique(a["confidence"]).tolist()}' if 'confidence' in a.dtype.names else ''))


if __name__ == '__main__':
    main()
