#!/usr/bin/env python3
"""Tüm tur boyunca yaya güvenliği: her yaya geçidi geçişinde ego hızı, yaya geçidi modülünün DUR kararı ve
yayaların araç gövdesine (base_link: arka aks; gövde x -1.1..3.9 m, y ±0.95 m) en yakın uzaklığı.
Kullanım: analiz_yaya_tur.py BAG"""
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
CW = {0: (27.4, -303.8), 1: (160.4, -266.6), 2: (-215.6, -34.4), 3: (-51.7, 1624.7), 4: (-189.4, -288.5), 5: (-113.3, -312.1)}
FX0, FX1, HW = -1.1, 3.9, 0.95
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/localization/kinematic_state", "/perception/object_recognition/objects",
                                              "/planning/planning_factors/crosswalk", "/diagnostics"]))
t0 = None; ego = None; passes = {}; cw_stop = False; collisions = []
def body_dist(lx, ly):
    dx = max(FX0 - lx, 0.0, lx - FX1); dy = max(abs(ly) - HW, 0.0)
    return math.hypot(dx, dy)
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t
    m = deserialize_message(d, get_message(ty[tp]))
    if tp == "/localization/kinematic_state":
        q = m.pose.pose.orientation
        ego = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), m.twist.twist.linear.x)
        for k, (cx, cy) in CW.items():
            dd = math.hypot(ego[0] - cx, ego[1] - cy)
            if dd < 25:
                p = passes.setdefault((k, int((t - t0) // 60)), dict(t=t - t0, dmin=99, vmin=99, vat=None, stop=False, ped_body=99, peds=0))
                p["dmin"] = min(p["dmin"], dd); p["vmin"] = min(p["vmin"], ego[3] * 3.6)
                if dd < 4 and p["vat"] is None: p["vat"] = round(ego[3] * 3.6, 1)
                if cw_stop: p["stop"] = True
    elif tp.endswith("crosswalk"):
        cw_stop = any(f.behavior == 3 for f in m.factors)
    elif tp.endswith("objects") and ego is not None:
        c, s = math.cos(ego[2]), math.sin(ego[2])
        for o in m.objects:
            lab = max(o.classification, key=lambda q: q.probability).label if o.classification else -1
            if lab != 7: continue
            p_ = o.kinematics.initial_pose_with_covariance.pose.position
            dx, dy = p_.x - ego[0], p_.y - ego[1]; lx, ly = c * dx + s * dy, -s * dx + c * dy
            bd = body_dist(lx, ly)
            for k, (cx, cy) in CW.items():
                if math.hypot(ego[0] - cx, ego[1] - cy) < 25:
                    p = passes.get((k, int((t - t0) // 60)))
                    if p: p["ped_body"] = min(p["ped_body"], bd); p["peds"] += 1
            if bd < 0.3 and abs(ego[3]) > 0.3:
                collisions.append((round(t - t0, 1), round(lx, 1), round(ly, 1), round(ego[3] * 3.6, 1)))
    elif tp == "/diagnostics":
        for st in m.status:
            lv = st.level[0] if isinstance(st.level, bytes) else st.level
            if "collision_detector" in st.name and lv > 0:
                collisions.append((round(t - t0, 1), "collision_detector", st.message[:40], ""))
print("geçit  t(s)    en yakın  en düşük hız  geçide 4m'de hız  DUR kararı  yaya-gövde en yakın")
for (k, _), p in sorted(passes.items(), key=lambda kv: kv[1]["t"]):
    print(f"  {k}   {p['t']:6.1f}  {p['dmin']:5.1f} m   {p['vmin']:5.1f} km/sa      {p['vat']}            {'evet' if p['stop'] else 'hayır':5s}"
          f"       {p['ped_body'] if p['peds'] else '-'} m ({p['peds']} yaya görüntüsü)")
print("hareket hâlindeyken gövdeye 0.3 m'den yakın yaya / çarpışma dedektörü:", collisions[:20] if collisions else "YOK")
