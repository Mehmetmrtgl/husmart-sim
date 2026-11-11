#!/usr/bin/env python3
"""Bir konum çevresinde: ego - planlanan yörünge uzaklığı (izleme hatası) ve yörüngenin eğriliği / hızı.
Kullanım: analiz_uturn.py BAG x,y [yarıçap_m]"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]; X, Y = map(float, sys.argv[2].split(",")); RAD = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/localization/kinematic_state", "/planning/scenario_planning/trajectory",
                                              "/vehicle/status/steering_status", "/control/command/control_cmd",
                                              "/api/planning/steering_factors", "/api/planning/velocity_factors"]))
traj = None; steer = 0.0; cmd = (0, 0); t0 = None; rows = []; factors = {}
def seg_dist(P, x, y):
    a, b = P[:-1], P[1:]; ab = b - a
    t = np.clip(((np.array([x, y]) - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
    q = a + t[:, None] * ab; d = np.linalg.norm(q - [x, y], axis=1); k = int(d.argmin())
    cr = ab[k, 0] * (y - a[k, 1]) - ab[k, 1] * (x - a[k, 0])
    return (1 if cr >= 0 else -1) * d[k], k
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t
    m = deserialize_message(d, get_message(ty[tp]))
    if tp.endswith("trajectory"):
        traj = (np.array([[p.pose.position.x, p.pose.position.y] for p in m.points]),
                np.array([p.longitudinal_velocity_mps for p in m.points]),
                np.array([p.front_wheel_angle_rad for p in m.points]) if hasattr(m.points[0], "front_wheel_angle_rad") else None)
    elif tp.endswith("steering_status"):
        steer = m.steering_tire_angle
    elif tp.endswith("control_cmd"):
        cmd = (m.lateral.steering_tire_angle, m.longitudinal.velocity)
    elif tp.endswith("_factors"):
        p0 = None
        for f in m.factors:
            factors.setdefault(round(t - t0), set()).add((tp.split("/")[-1][:8], f.behavior, f.status))
    else:
        p = m.pose.pose.position
        if math.hypot(p.x - X, p.y - Y) < RAD and traj is not None and len(traj[0]) > 2:
            e, k = seg_dist(traj[0], p.x, p.y)
            # yörüngenin bu noktadaki eğrilik yarıçapı (3 nokta)
            P = traj[0]; i = min(max(k, 1), len(P) - 2)
            a, b, c = P[i - 1], P[i], P[i + 1]
            A = np.linalg.norm(b - a); B = np.linalg.norm(c - b); C = np.linalg.norm(c - a)
            area = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2
            R = (A * B * C) / (4 * area) if area > 1e-6 else float("inf")
            rows.append((t - t0, p.x, p.y, m.twist.twist.linear.x, e, R, steer, cmd[0]))
for row in rows[::10]:
    print("t %7.1f (%7.1f,%7.1f) v %4.1f km/h | ego-yörünge %+.2f m | yörünge R %6.1f m | direksiyon %+.3f komut %+.3f rad" %
          (row[0], row[1], row[2], row[3] * 3.6, row[4], row[5], row[6], row[7]))
if rows:
    E = np.array(rows); print("izleme hatası |e| medyan %.2f max %.2f m; en dar yörünge R %.1f m; en büyük direksiyon %.3f rad" %
                              (np.median(np.abs(E[:, 4])), np.abs(E[:, 4]).max(), E[:, 5].min(), np.abs(E[:, 6]).max()))
    ts = sorted(set(int(x) for x in E[:, 0]))
    for s in ts[::3]:
        if s in factors: print("  t %d faktörler: %s" % (s, sorted(factors[s])[:6]))
