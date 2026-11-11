#!/usr/bin/env python3
"""Yaya geçidi olayı: crosswalk / run_out planlama faktörleri (TTC/TTV), yayalar ve ego, verilen bag zaman aralığında.
Kullanım: analiz_yaya.py BAG t0 t1   (t0,t1: bag başından saniye)"""
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, T0, T1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ["/localization/kinematic_state", "/planning/planning_factors/crosswalk", "/planning/planning_factors/run_out",
        "/perception/object_recognition/objects", "/control/command/control_cmd", "/api/planning/velocity_factors"]
r.set_filter(rosbag2_py.StorageFilter(topics=want))
start = None; ego = None; last_print = -1e9; st = {}
LAB = {0: "?", 1: "car", 2: "truck", 3: "bus", 4: "trailer", 5: "moto", 6: "bike", 7: "ped"}
def fmt_pf(m):
    out = []
    for f in m.factors:
        sf = []
        for s in f.safety_factors.factors:
            sf.append(f"{'güvenli' if s.is_safe else 'GÜVENSİZ'} ttc[{s.ttc_begin:.1f},{s.ttc_end:.1f}]")
        d = [round(c.distance, 1) for c in f.control_points][:2]
        out.append(f"beh={f.behavior} '{f.detail[:40]}' mesafe={d} {sf[:2]}")
    return out
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if start is None: start = t
    rel = t - start
    if rel < T0: continue
    if rel > T1: break
    m = deserialize_message(d, get_message(ty[tp]))
    if tp == "/localization/kinematic_state":
        q = m.pose.pose.orientation
        ego = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), m.twist.twist.linear.x)
        if rel - last_print >= 0.3:
            last_print = rel
            peds = st.get("obj", [])
            print(f"t {rel:6.1f} ego ({ego[0]:.1f},{ego[1]:.1f}) v {ego[3]*3.6:4.1f} km/h a_cmd {st.get('acc', 0):+.2f} | "
                  f"crosswalk {st.get('cw', [])} | run_out {st.get('ro', [])[:1]} | vf {st.get('vf', [])} | yaya {peds}")
    elif tp.endswith("crosswalk"):
        st["cw"] = fmt_pf(m)
    elif tp.endswith("run_out"):
        st["ro"] = fmt_pf(m)
    elif tp.endswith("control_cmd"):
        st["acc"] = m.longitudinal.acceleration
    elif tp.endswith("velocity_factors"):
        st["vf"] = sorted({(f.behavior, round(f.distance, 1)) for f in m.factors if f.distance < 80})[:3]
    elif tp.endswith("objects") and ego is not None:
        c, s = math.cos(ego[2]), math.sin(ego[2]); L = []
        for o in m.objects:
            lab = max(o.classification, key=lambda q: q.probability).label if o.classification else -1
            if lab != 7: continue
            p = o.kinematics.initial_pose_with_covariance.pose.position
            dx, dy = p.x - ego[0], p.y - ego[1]; lx, ly = c * dx + s * dy, -s * dx + c * dy
            if math.hypot(lx, ly) < 25:
                vv = o.kinematics.initial_twist_with_covariance.twist.linear
                L.append((round(lx, 1), round(ly, 1), round(math.hypot(vv.x, vv.y), 1)))
        st["obj"] = sorted(L)[:4]
