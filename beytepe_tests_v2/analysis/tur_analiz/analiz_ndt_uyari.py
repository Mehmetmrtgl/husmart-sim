#!/usr/bin/env python3
"""NDT scan_matching_status uyarı dönemleri (WARN/ERROR): her dönemin süresi, mesajları, konumu, hızı.
Süre = ilk uyarı mesajından sonraki ilk OK mesajına kadar (tanılama grafiğinin o birimi uyarıda gördüğü süre).
Kullanım: analiz_ndt_uyari.py BAG [BAG2 ...]"""
import sys, collections
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/diagnostics", "/localization/kinematic_state"]))
    t0 = None; ego = (0.0, 0.0, 0.0); seq = []
    while r.has_next():
        tp, d, t = r.read_next(); t *= 1e-9
        if t0 is None: t0 = t
        m = deserialize_message(d, get_message(ty[tp]))
        if tp == "/localization/kinematic_state":
            ego = (m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x); continue
        for s in m.status:
            if "ndt_scan_matcher" in s.name and s.name.endswith("scan_matching_status"):
                lv = s.level[0] if isinstance(s.level, bytes) else s.level
                seq.append((t - t0, int(lv), s.message, ego))
    name = bag.rstrip("/").split("/")[-1]
    if len(seq) < 2:
        print(f"== {name}: NDT teşhisi yok"); continue
    rate = (len(seq) - 1) / (seq[-1][0] - seq[0][0])
    first_ok = next((i for i, x in enumerate(seq) if x[1] == 0), None)
    eps = []; i = first_ok if first_ok is not None else len(seq)
    while i < len(seq):
        if seq[i][1] == 0: i += 1; continue
        j = i; mx = 0; msgs = collections.Counter()
        while j < len(seq) and seq[j][1] != 0:
            mx = max(mx, seq[j][1]); msgs[seq[j][2][:60]] += 1; j += 1
        end = seq[j][0] if j < len(seq) else seq[-1][0]
        eps.append((seq[i][0], end - seq[i][0], j - i, mx, msgs, seq[i][3], j >= len(seq)))
        i = j
    lv_all = collections.Counter(x[1] for x in seq)
    print(f"== {name}: {len(seq)} NDT teşhis mesajı ({rate:.1f} Hz), seviye dağılımı {dict(sorted(lv_all.items()))}, "
          f"ilk OK t={seq[first_ok][0]:.1f} s" if first_ok is not None else f"== {name}: hiç OK yok")
    dur = [e[1] for e in eps if not e[6]]
    print(f"   uyarı dönemi: {len(eps)}  | süre <=0.15 s: {sum(d <= 0.15 for d in dur)}, 0.15-0.35: {sum(0.15 < d <= 0.35 for d in dur)}, "
          f"0.35-0.6: {sum(0.35 < d <= 0.6 for d in dur)}, 0.6-1.0: {sum(0.6 < d <= 1.0 for d in dur)}, >1.0: {sum(d > 1.0 for d in dur)}")
    for t, dd, n, mx, msgs, e, tail in eps:
        top = "; ".join(f"{k} x{v}" for k, v in msgs.most_common(3))
        print(f"   t={t:7.1f} s  süre {dd:5.2f} s ({n} mesaj) seviye {mx}  konum ({e[0]:7.1f},{e[1]:7.1f}) {e[2] * 3.6:5.1f} km/sa"
              f"{'  [bag sonu]' if tail else ''} | {top}")
