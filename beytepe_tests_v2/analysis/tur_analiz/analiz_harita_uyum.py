#!/usr/bin/env python3
"""Tarama-harita uyumu: verilen bag anlarında lokalizasyon bulutunu (base_link) NDT pozuyla haritaya taşı, her noktanın
en yakın harita noktasına uzaklığını ölç. Haritada karşılığı olmayan nokta oranı yüksekse sahne ile harita uyuşmuyor.
Ayrıca karşılıksız noktaların araç çevresindeki yönünü (ön/arka/sol/sağ) ve yüksekliğini verir.
Kullanım: analiz_harita_uyum.py BAG KARO_KLASORU t1 t2 ..."""
import sys, os, math, glob
import numpy as np
from scipy.spatial import cKDTree
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

bag, tiles_dir = sys.argv[1], sys.argv[2]; T = [float(x) for x in sys.argv[3:]]
def load_tile(p):
    with open(p, "rb") as f: raw = f.read()
    i = raw.index(b"\nDATA binary\n") + len(b"\nDATA binary\n")
    return np.frombuffer(raw[i:], dtype="<f4").reshape(-1, 3)
meta = {}
for p in glob.glob(os.path.join(tiles_dir, "pointcloud_map", "*.pcd")):
    _, _, ix, iy = os.path.basename(p)[:-4].split("_"); meta[(int(ix), int(iy))] = p
cache = {}
def map_near(x, y, R=70.0):
    pts = []
    for ix in range(int(math.floor((x - R) / 50)), int(math.floor((x + R) / 50)) + 1):
        for iy in range(int(math.floor((y - R) / 50)), int(math.floor((y + R) / 50)) + 1):
            if (ix, iy) in meta:
                if (ix, iy) not in cache: cache[(ix, iy)] = load_tile(meta[(ix, iy)])
                pts.append(cache[(ix, iy)])
    return np.concatenate(pts) if pts else np.zeros((0, 3), np.float32)
def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/localization/util/downsample/pointcloud", "/localization/pose_estimator/pose_with_covariance"]))
t0 = None; clouds = {}; poses = {}; frame = None
st = lambda h: round(h.stamp.sec + h.stamp.nanosec * 1e-9, 3)
want = sorted(T); k = 0
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if t0 is None: t0 = t
    rel = t - t0
    if rel < want[0] - 1 or rel > want[-1] + 1: continue
    if not any(abs(rel - w) < 0.6 for w in want): continue
    m = deserialize_message(d, get_message(ty[tp]))
    if tp.endswith("pointcloud"):
        a = pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
        clouds[st(m.header)] = (rel, np.stack([np.asarray(a[c], float) for c in ("x", "y", "z")], 1)); frame = m.header.frame_id
    else:
        p, q = m.pose.pose.position, m.pose.pose.orientation; poses[st(m.header)] = (np.array([p.x, p.y, p.z]), q2R(q.x, q.y, q.z, q.w))
print(f"bulut çerçevesi: {frame}")
print("  t(s)   konum            nokta  karşılıksız(>1m)  >0.5m   medyan(m) | karşılıksızların yönü: ön arka sol sağ | yükseklik medyan")
done = set()
for s, (rel, P) in sorted(clouds.items()):
    w = min(want, key=lambda x: abs(x - rel))
    if w in done or s not in poses: continue
    done.add(w); pos, R = poses[s]
    M = map_near(pos[0], pos[1]); tree = cKDTree(M)
    W = (R @ P.T).T + pos
    dist, _ = tree.query(W, k=1)
    bad = dist > 1.0; side = P[bad]
    dirs = [int(((side[:, 0] > 0) & (np.abs(side[:, 0]) >= np.abs(side[:, 1]))).sum()), int(((side[:, 0] < 0) & (np.abs(side[:, 0]) >= np.abs(side[:, 1]))).sum()),
            int(((side[:, 1] > 0) & (np.abs(side[:, 1]) > np.abs(side[:, 0]))).sum()), int(((side[:, 1] < 0) & (np.abs(side[:, 1]) > np.abs(side[:, 0]))).sum())]
    print(f"{rel:7.1f}  ({pos[0]:7.1f},{pos[1]:7.1f})  {len(P):5d}   {100 * bad.mean():5.1f}%         {100 * (dist > 0.5).mean():5.1f}%   {np.median(dist):.2f}     | {dirs} | {np.median(side[:, 2]) if bad.any() else float('nan'):.2f} m")
