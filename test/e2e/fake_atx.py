#!/usr/bin/env python3
"""Send synthetic ATX (UDP protocol 4.7) packets so the driver can run without a lidar.

Every 10th channel is a no-return (distance 0); the rest sit at 10 m. Frames alternate the
mirror-face bit like the real sensor.
"""
import argparse
import socket
import struct
import time

LASERS, BLOCKS = 116, 2
DIST_UNIT_MM = 4


def packet(frame_id, az_deg, t_us, seq):
    pre = struct.pack('<HBBBB', 0xFFEE, 4, 7, 0, 0)
    header = struct.pack('<BBBBBB', LASERS, BLOCKS, 0, DIST_UNIT_MM, 1, 0x01)  # status: has seq num
    body = b''
    for b in range(BLOCKS):
        body += struct.pack('<HB', int((az_deg + 0.05 * b) * 256), 0)
        for ch in range(LASERS):
            dist = 0 if ch % 10 == 0 else int(10.0 / (DIST_UNIT_MM / 1000.0))
            body += struct.pack('<HBB', dist, 50 + ch, 2)
    crc = struct.pack('<I', 0)
    sec, usec = divmod(t_us, 1_000_000)
    utc = bytes([0, 0]) + struct.pack('>I', sec)
    tail = struct.pack('<HB7sB7shIBB6s', 0, 0, b'\0' * 7, frame_id, b'\0' * 7, 600, usec, 0x37, 0, utc)
    return pre + header + body + crc + tail + struct.pack('<I', seq) + b'\0' * 12


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=2368)
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--packets', type=int, default=200)
    a = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    for f in range(a.frames + 1):
        t0 = time.time()
        for p in range(a.packets):
            az = 30.0 + 120.0 * p / a.packets
            s.sendto(packet(f % 2, az, int((t0 + p * 1e-4) * 1e6), seq), ('127.0.0.1', a.port))
            seq += 1
        time.sleep(max(0.0, 0.1 - (time.time() - t0)))


if __name__ == '__main__':
    main()
