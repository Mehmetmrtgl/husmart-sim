#!/usr/bin/env python3
"""NDT olayı: verilen bag zaman aralığında NDT döngüleri (iterasyon, NVTL, TP, süre), NDT konumu - yer gerçeği,
ego hızı / yaw hızı ve ndt_scan_matcher teşhis anahtarları.
Kullanım: analiz_ndt_olay.py BAG t0 t1"""
import sys, math, bisect
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, T0, T1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
ANT = np.array([0.40, 0.0, 1.82])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ["/localization/pose_estimator/pose_with_covariance", "/localization/pose_estimator/iteration_num",
        "/localization/pose_estimator/nearest_voxel_transformation_likelihood", "/localization/pose_estimator/transform_probability",
        "/localization/pose_estimator/exe_time_ms", "/localization/kinematic_state", "/sensing/gnss/pose_with_covariance",
        "/diagnostics", "/localization/util/downsample/pointcloud"]
r.set_filter(rosbag2_py.StorageFilter(topics=want))
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
t0 = None; gt = []; ekf = []; rows = {}; diag = []; npts = {}
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t
    rel = t - t0
    if rel < T0 - 5 or rel > T1 + 5: continue
    m = deserialize_message(d, get_message(ty[tp]))
    if tp == "/sensing/gnss/pose_with_covariance":
        p = m.pose.pose.position; gt.append((st(m.header), p.x, p.y, p.z))
    elif tp == "/localization/kinematic_state":
        q = m.pose.pose.orientation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ekf.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif tp == "/localization/pose_estimator/pose_with_covariance":
        s = round(st(m.header), 3); p = m.pose.pose.position; q = m.pose.pose.orientation
        rows.setdefault(s, {})["pose"] = (p.x, p.y, p.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        rows[s]["rel"] = rel
    elif tp.endswith("iteration_num") or tp.endswith("likelihood") or tp.endswith("probability") or tp.endswith("exe_time_ms"):
        s = round(m.stamp.sec + m.stamp.nanosec * 1e-9 if hasattr(m, "stamp") else st(m.header), 3)
        rows.setdefault(s, {})[tp.split("/")[-1][:6]] = m.data
    elif tp == "/localization/util/downsample/pointcloud":
        npts[round(st(m.header), 3)] = m.width * m.height
    elif tp == "/diagnostics":
        for s_ in m.status:
            if "ndt_scan_matcher" in s_.name and "scan_matching_status" in s_.name:
                lv = s_.level[0] if isinstance(s_.level, bytes) else s_.level
                kv = {k.key: k.value for k in s_.values}
                diag.append((rel, lv, s_.message[:70], kv))
gt = np.array(gt); E = np.array(ekf)
def gt_base(s):
    if len(gt) < 2: return None
    i = bisect.bisect_left(gt[:, 0], s); i = min(max(i, 1), len(gt) - 1)
    a, b = gt[i - 1], gt[i]; w = (s - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
    p = a[1:3] + w * (b[1:3] - a[1:3])
    j = min(bisect.bisect_left(E[:, 0], s), len(E) - 1); yaw = E[j, 3]
    return p - np.array([math.cos(yaw) * ANT[0] - math.sin(yaw) * ANT[1], math.sin(yaw) * ANT[0] + math.cos(yaw) * ANT[1]])
print(" bag_t   iter  NVTL   TP    süre  nokta | NDT-gerçek: yatay  boyuna  yanal | hız km/h  yaw hızı")
for s in sorted(rows):
    rw = rows[s]
    if "pose" not in rw or not (T0 <= rw["rel"] <= T1): continue
    g = gt_base(s); j = min(bisect.bisect_left(E[:, 0], s), len(E) - 1)
    if g is None: continue
    dx, dy = rw["pose"][0] - g[0], rw["pose"][1] - g[1]; yaw = E[j, 3]
    lon = math.cos(yaw) * dx + math.sin(yaw) * dy; lat = -math.sin(yaw) * dx + math.cos(yaw) * dy
    print(f"{rw['rel']:7.2f} {rw.get('iterat', -1):4.0f} {rw.get('neares', float('nan')):5.2f} {rw.get('transf', float('nan')):5.2f} {rw.get('exe_ti', float('nan')):5.1f} {npts.get(s, -1):5d} |"
          f"   {math.hypot(dx, dy):5.2f}  {lon:+5.2f}  {lat:+5.2f} | {E[j,4]*3.6:5.1f}  {E[j,5]:+.3f}")
print("\nteşhis (seviye>0):")
seen = set()
for rel, lv, msg, kv in diag:
    if lv > 0 and T0 - 2 <= rel <= T1:
        key = (round(rel, 1), msg)
        if key in seen: continue
        seen.add(key)
        keep = {k: v for k, v in kv.items() if any(w in k for w in ("oscillation", "iteration", "nearest_voxel", "transform_probability", "distance_initial", "execution_time", "skipping", "local_optimal", "is_"))}
        print(f"  {rel:.1f} L{lv} {msg} | {keep}")
