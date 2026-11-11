#!/usr/bin/env python3
"""Sert fren olaylarının zaman çizelgesi: verilen konumlarda aracın durduğu anı bulur ve öncesini/sonrasını döker.
Kullanım: analiz_fren.py BAG x1,y1 [x2,y2 ...]"""
import sys, math, collections
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
targets = [tuple(map(float, a.split(","))) for a in sys.argv[2:]]
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9

def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while r.has_next():
        tp, d, t = r.read_next()
        yield tp, deserialize_message(d, get_message(ty[tp])), t * 1e-9

# 1. geçiş: EKF ve hız -> olay anları
ek = []
for tp, m, t in reader(["/localization/kinematic_state"]):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    ek.append((t, p.x, p.y, m.twist.twist.linear.x, yaw))
EK = np.array(ek); T0 = EK[0, 0]
events = []
for (x, y) in targets:
    d = np.hypot(EK[:, 1] - x, EK[:, 2] - y)
    cand = np.where((d < 3.0) & (np.abs(EK[:, 3]) < 0.05))[0]
    if not len(cand):
        cand = [int(d.argmin())]
    i = int(cand[0]); events.append((x, y, EK[i, 0]))
    print(f"olay ({x},{y}): durma anı bag t={EK[i,0]-T0:.2f} s (sim {EK[i,0]:.2f})")

W0, W1 = 9.0, 3.0
def inwin(t):
    for k, (_, _, te) in enumerate(events):
        if te - W0 <= t <= te + W1:
            return k
    return None

rows = collections.defaultdict(list)
logs = collections.defaultdict(list)
diag = collections.defaultdict(collections.Counter)
topics = ["/localization/kinematic_state", "/vehicle/status/velocity_status", "/control/command/control_cmd",
          "/planning/scenario_planning/trajectory", "/planning/scenario_planning/max_velocity_candidates",
          "/perception/object_recognition/objects", "/api/fail_safe/mrm_state", "/rosout", "/diagnostics",
          "/sensing/ins/mti7/pose_with_covariance", "/api/planning/velocity_factors"]
last = {}
for tp, m, t in reader(topics):
    k = inwin(t)
    if k is None:
        continue
    if tp == "/localization/kinematic_state":
        p = m.pose.pose.position; q = m.pose.pose.orientation
        last["ego"] = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), m.twist.twist.linear.x)
    elif tp == "/sensing/ins/mti7/pose_with_covariance":
        last["ins"] = (m.pose.pose.position.x, m.pose.pose.position.y)
    elif tp == "/vehicle/status/velocity_status":
        last["v"] = m.longitudinal_velocity
    elif tp == "/control/command/control_cmd":
        last["cmd"] = (m.longitudinal.velocity, m.longitudinal.acceleration)
        # satır: 0.1 s'de bir
        if not rows[k] or t - rows[k][-1][0] >= 0.1:
            rows[k].append((t, dict(last)))
    elif tp == "/planning/scenario_planning/trajectory":
        pts = m.points
        if pts and "ego" in last:
            ex, ey = last["ego"][0], last["ego"][1]
            s = 0.0; stop = None; prev = None; vmin = 99
            for pt in pts:
                pp = pt.pose.position
                if prev is not None:
                    s += math.hypot(pp.x - prev[0], pp.y - prev[1])
                prev = (pp.x, pp.y)
                if s < 40: vmin = min(vmin, pt.longitudinal_velocity_mps)
                if stop is None and pt.longitudinal_velocity_mps < 0.01 and s > 0.0:
                    stop = (round(s, 1), round(pp.x, 1), round(pp.y, 1))
            last["traj"] = (round(vmin, 2), stop)
    elif tp == "/planning/scenario_planning/max_velocity_candidates":
        last["vlim"] = [(c.sender[-30:] if hasattr(c, "sender") else "", round(c.max_velocity, 2)) for c in m.velocity_limits] if hasattr(m, "velocity_limits") else str(m)[:120]
    elif tp == "/perception/object_recognition/objects":
        if "ego" in last:
            ex, ey, yaw, _ = last["ego"]; c, s_ = math.cos(yaw), math.sin(yaw)
            near = []
            for o in m.objects:
                pp = o.kinematics.initial_pose_with_covariance.pose.position
                dx, dy = pp.x - ex, pp.y - ey; lx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
                if -2 < lx < 40 and abs(ly) < 6:
                    lab = max(o.classification, key=lambda q: q.probability).label if o.classification else -1
                    vv = o.kinematics.initial_twist_with_covariance.twist.linear
                    near.append((round(lx, 1), round(ly, 1), lab, round(math.hypot(vv.x, vv.y), 1)))
            last["obj"] = sorted(near)[:4]
    elif tp == "/api/planning/velocity_factors":
        last["vf"] = sorted({(f.behavior, round(f.distance, 1), f.status) for f in m.factors if f.distance < 60})[:3]
    elif tp == "/api/fail_safe/mrm_state":
        last["mrm"] = (m.state, m.behavior)
    elif tp == "/rosout":
        if m.level >= 30 or any(w in m.msg.lower() for w in ("stop", "crosswalk", "obstacle", "emergency", "brake")):
            logs[k].append((round(t - events[k][2], 2), m.level, m.name[-45:], m.msg[:160]))
    elif tp == "/diagnostics":
        for s in m.status:
            lv = s.level[0] if isinstance(s.level, bytes) else s.level
            if lv > 0:
                diag[k][(s.name[:70], s.message[:80], lv)] += 1

LAB = {0: "?", 1: "car", 2: "truck", 3: "bus", 4: "trailer", 5: "motorcycle", 6: "bicycle", 7: "pedestrian"}
for k, (x, y, te) in enumerate(events):
    print(f"\n================ olay {k} ({x},{y})  durma bag t={te-T0:.1f}")
    for t, d in rows[k]:
        if int(round((t - te) * 10)) % 3 and abs(t - te) > 1.5:
            continue
        e = d.get("ego", (0, 0, 0, 0)); cmd = d.get("cmd", (0, 0)); tr = d.get("traj"); ins = d.get("ins")
        err = math.hypot(e[0] - ins[0], e[1] - ins[1]) if ins else float("nan")
        objs = " ".join(f"[{o[0]},{o[1]} {LAB.get(o[2], o[2])} {o[3]}m/s]" for o in d.get("obj", []))
        print(f"{t-te:+6.1f} v={d.get('v', 0)*3.6:5.1f} km/h cmd v={cmd[0]*3.6:5.1f} a={cmd[1]:+5.2f} | traj vmin40={tr[0] if tr else '-'} dur={tr[1] if tr else '-'}"
              f" | mrm={d.get('mrm')} | gerekçe {d.get('vf')} | nesne {objs}")
    print("  vlim:", rows[k][-1][1].get("vlim") if rows[k] else None)
    print("  log:")
    for l in logs[k][:25]:
        print("   ", l)
    print("  diag (seviye>0):")
    for (n, msg, lv), c in diag[k].most_common(12):
        print(f"    {c:4d} L{lv} {n} | {msg}")
