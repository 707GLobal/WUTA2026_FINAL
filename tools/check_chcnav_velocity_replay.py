"""Replay the acceleration INS samples into an isolated bridge, without hardware."""
import csv
import importlib.util
import math
from pathlib import Path
import re

workspace = Path(__file__).resolve().parents[1]
test_path = workspace / 'humble-chcnav-cgi_ros2pkg/src/chcnav/test/test_fsd_bridge_velocity.py'
spec = importlib.util.spec_from_file_location('bridge_test', test_path)
test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(test)
log = workspace / 'humble-chcnav-cgi_ros2pkg/src/chcnav/log/20261011_165340/devpvt.log'
chunks = [x for x in re.split(r'(?=\[2026-)', log.read_text().split('\x00')[0])
          if x.startswith('[2026-') and
          '2026-10-11 16:54:14.007' <= x[1:27] <= '2026-10-11 16:54:17.148']
fixture = test.bridge.__wrapped__()
bridge = next(fixture)
rows = []
try:
    for tag, chunk in enumerate(chunks, 100):
        enu = tuple(map(float, re.search(
            r'enu_vel\s*= e ([\d.-]+)\s+n ([\d.-]+)\s+u ([\d.-]+)', chunk).groups()))
        roll, pitch, yaw = tuple(map(float, re.search(
            r'rpy\s*= roll ([\d.-]+)\s+pitch ([\d.-]+)\s+yaw ([\d.-]+)', chunk).groups()))
        msg = test.measurement(yaw, enu, tag)
        msg.roll, msg.pitch = roll, pitch
        odom, vel = test.send(bridge, msg)
        v = vel.twist.linear
        src_norm = math.sqrt(sum(x*x for x in enu))
        dst_norm = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        assert abs(src_norm - dst_norm) < 1e-5
        assert odom.twist.twist == vel.twist
        rows.append(dict(time=chunk[1:27], device_yaw_deg=yaw,
                         enu_e_mps=enu[0], enu_n_mps=enu[1], enu_u_mps=enu[2],
                         body_vx_mps=v.x, body_vy_mps=v.y, body_vz_mps=v.z,
                         planar_speed_mps=math.hypot(v.x, v.y),
                         direction_from_heading_deg=math.degrees(math.atan2(v.y, v.x))))
finally:
    fixture.close()
out = workspace / 'docs/chcnav_velocity_replay_165355.csv'
with out.open('w') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
print('Replayed', len(rows), 'frames; output:', out)
print('Maximum planar speed:', max(r['planar_speed_mps'] for r in rows))
print('First moving sample:', next(r for r in rows if r['planar_speed_mps'] > 1))
print('Peak speed sample:', max(rows, key=lambda r:r['planar_speed_mps']))
