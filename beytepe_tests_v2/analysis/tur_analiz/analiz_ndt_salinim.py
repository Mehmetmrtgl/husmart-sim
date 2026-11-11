#!/usr/bin/env python3
"""Tüm tur boyunca NDT salınım sayacı ve iterasyon sayısı: yüksek değerlerin konumları (sıcak noktalar).
Kullanım: analiz_ndt_salinim.py BAG [BAG2 ...]"""
import sys, math, collections
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/diagnostics", "/localization/kinematic_state"]))
    t0 = None; ego = (0.0, 0.0, 0.0, 0.0); hits = []; n = 0; osc_hist = collections.Counter()
    while r.has_next():
        tp, d, t = r.read_next(); t *= 1e-9
        if t0 is None: t0 = t
        m = deserialize_message(d, get_message(ty[tp]))
        if tp == "/localization/kinematic_state":
            ego = (m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x, m.twist.twist.angular.z); continue
        for s in m.status:
            if "scan_matching_status" not in s.name: continue
            kv = {k.key: k.value for k in s.values}
            try:
                osc = int(kv.get("local_optimal_solution_oscillation_num", "0")); it = int(kv.get("iteration_num", "0"))
            except ValueError:
                continue
            n += 1; osc_hist[min(osc, 20) // 5 * 5] += 1
            if osc >= 8 or it >= 15:
                hits.append((round(t - t0, 1), osc, it, round(ego[0]), round(ego[1]), round(ego[2] * 3.6), round(ego[3], 3)))
    print(f"== {bag.split('/')[-1]}: {n} NDT döngüsü; salınım sayacı dağılımı {dict(sorted(osc_hist.items()))}")
    cl = collections.defaultdict(list)
    for h in hits: cl[(h[3] // 25 * 25, h[4] // 25 * 25)].append(h)
    for k, v in sorted(cl.items(), key=lambda kv: -len(kv[1])):
        mx = max(v, key=lambda h: h[1])
        print(f"   bölge ~({k[0]},{k[1]}): {len(v)} döngü, en yüksek salınım {mx[1]} (iterasyon {mx[2]}) t={mx[0]} s, hız {mx[5]} km/sa, yaw hızı {mx[6]}")
