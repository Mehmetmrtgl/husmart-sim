#!/usr/bin/env python3
"""Yaya geçidi modülü karar çizelgesi: verilen bag aralığında /planning/planning_factors/crosswalk (davranış, ayrıntı,
kontrol noktası uzaklığı, güvenlik etkenleri: nesne, ttc_begin/ttc_end, güvenli mi) ve ego hızı.
Kullanım: analiz_yaya_karar.py BAG t0 t1"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, T0, T1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/planning/planning_factors/crosswalk", "/localization/kinematic_state"]))
t0 = None; v = 0.0; last = None
BEH = {0: "UNKNOWN", 1: "NONE", 2: "SLOW_DOWN", 3: "STOP"}
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t
    rel = t - t0
    if rel < T0 or rel > T1: continue
    m = deserialize_message(d, get_message(ty[tp]))
    if tp == "/localization/kinematic_state":
        v = m.twist.twist.linear.x; continue
    rows = []
    for f in m.factors:
        cps = [round(c.distance, 1) for c in f.control_points]
        sf = []
        for s in f.safety_factors.factors:
            p = s.points[0] if len(s.points) else None
            sf.append(f"obj tip={s.type} ttc=({s.ttc_begin:.2f},{s.ttc_end:.2f}) güvenli={s.is_safe}" + (f" @({p.x:.1f},{p.y:.1f})" if p else ""))
        rows.append(f"{BEH.get(f.behavior, f.behavior)} '{f.detail[:60]}' kn={cps} | " + "; ".join(sf[:3]))
    key = tuple(rows)
    if key != last:
        print(f"{rel:7.2f} v={v * 3.6:5.1f} km/sa | " + (" || ".join(rows) if rows else "(etken yok)"))
        last = key
