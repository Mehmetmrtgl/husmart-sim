"""FBX'teki yol yüzeyleri ile lanelet2 şeritlerini aynı kuşbakışı çizimde gösterir.
Kullanım: python3 -I road_vs_lanelet.py <fbx> <osm> <cx> <cy> <yarıçap> <çıktı.png> [işaret_x işaret_y ...]"""
import sys, re, math
import numpy as np
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fbx_check import read_fbx, child, props70, local_matrix, geometric_matrix
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

fbx, osm, cx, cy, R, out = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:6]), sys.argv[6]
marks = list(map(float, sys.argv[7:]))
ver, top = read_fbx(fbx); root = {n["name"]: n for n in top}
objs = root["Objects"]["children"]
geoms = {o["props"][0]: o for o in objs if o["name"] == "Geometry"}
models = {o["props"][0]: o for o in objs if o["name"] == "Model"}
mats = {o["props"][0]: o["props"][1].split("\x00")[0] for o in objs if o["name"] == "Material"}
parent, gm, mm = {}, {}, {}
for c in root["Connections"]["children"]:
    if c["props"][0] != "OO": continue
    a, b = c["props"][1], c["props"][2]
    if a in models and (b in models or b == 0): parent[a] = b
    if a in geoms and b in models: gm[a] = b
    if a in mats and b in models: mm.setdefault(b, []).append(mats[a])
def world(mid):
    M = np.eye(4); x = mid
    while x in models:
        M = local_matrix(props70(models[x])) @ M; x = parent.get(x, 0)
    return M
tris = {"road": [], "other": []}
for gid, g in geoms.items():
    if gid not in gm: continue
    mid = gm[gid]; names = mm.get(mid, [])
    V = np.asarray(child(g, "Vertices")["props"][0]).reshape(-1, 3)
    W = (np.c_[V, np.ones(len(V))] @ (world(mid) @ geometric_matrix(props70(models[mid]))).T)[:, :3]
    if not ((np.abs(W[:, 0] - cx) < R) & (np.abs(W[:, 1] - cy) < R)).any(): continue
    PVI = np.asarray(child(g, "PolygonVertexIndex")["props"][0])
    men = child(g, "LayerElementMaterial"); MI = np.asarray(child(men, "Materials")["props"][0]) if men else np.array([0])
    idx = PVI.copy(); ends = np.where(idx < 0)[0]; idx[ends] = -idx[ends] - 1; st = 0
    for pi, e in enumerate(ends):
        poly = idx[st:e + 1]; st = e + 1
        P = W[poly][:, :2]
        if (np.abs(P[:, 0] - cx) > R).all() or (np.abs(P[:, 1] - cy) > R).all(): continue
        mi = MI[pi] if len(MI) > 1 else MI[0]
        nm = names[mi] if mi < len(names) else "?"
        key = "road" if ("Asphalt" in nm or "Concrete1_Road" in nm) else ("other" if any(k in nm for k in ("Sidewalk", "Curb", "Gutter", "Marking")) else None)
        if key: tris[key].append(P)
# lanelet'ler (local_x/local_y)
txt = open(osm, encoding="utf-8").read()
nodes = {}
for m in re.finditer(r'<node id="(-?\d+)"[^>]*?(/>|>(.*?)</node>)', txt, re.S):
    body = m.group(3) or ""
    lx = re.search(r'k="local_x" v="([-0-9.eE]+)"', body); ly = re.search(r'k="local_y" v="([-0-9.eE]+)"', body)
    if lx and ly: nodes[m.group(1)] = (float(lx.group(1)), float(ly.group(1)))
ways = {m.group(1): [nodes[r] for r in re.findall(r'<nd ref="(-?\d+)"', m.group(2)) if r in nodes]
        for m in re.finditer(r'<way id="(-?\d+)"[^>]*>(.*?)</way>', txt, re.S)}
lls = []
for m in re.finditer(r'<relation id="(\d+)"[^>]*>(.*?)</relation>', txt, re.S):
    b = m.group(2)
    if 'v="lanelet"' not in b: continue
    L = re.search(r'type="way" ref="(-?\d+)" role="left"', b); Rr = re.search(r'type="way" ref="(-?\d+)" role="right"', b)
    if L and Rr: lls.append((m.group(1), ways.get(L.group(1), []), ways.get(Rr.group(1), [])))
fig, ax = plt.subplots(figsize=(10, 10))
ax.add_collection(PolyCollection(tris["road"], facecolor="#555555", edgecolor="none", label="3B model: asfalt/yol"))
ax.add_collection(PolyCollection(tris["other"], facecolor="#bbbbbb", edgecolor="none", alpha=0.7, label="3B model: kaldırım/bordür/çizgi"))
for lid, l, r in lls:
    if not l or not r: continue
    poly = np.array(l + r[::-1])
    if (np.abs(poly[:, 0] - cx) > R).all() or (np.abs(poly[:, 1] - cy) > R).all(): continue
    ax.fill(poly[:, 0], poly[:, 1], facecolor="none", edgecolor="#1f77b4", lw=1.0)
    c = poly.mean(0)
    if abs(c[0] - cx) < R and abs(c[1] - cy) < R: ax.text(c[0], c[1], lid[-3:], fontsize=6, color="#1f77b4")
for i in range(0, len(marks), 2):
    ax.plot(marks[i], marks[i + 1], "o", ms=9, color="#d62728"); ax.text(marks[i] + 2, marks[i + 1] + 2, "araç %d" % (i // 2 + 1), color="#d62728")
ax.plot([], [], color="#1f77b4", label="lanelet2 şeritleri (Autoware haritası)")
ax.set_xlim(cx - R, cx + R); ax.set_ylim(cy - R, cy + R); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.legend(loc="upper right", fontsize=8); ax.set_title("3B model yolu ve Autoware şeritleri, merkez (%.0f, %.0f)" % (cx, cy))
fig.savefig(out, dpi=110, bbox_inches="tight"); print("kaydedildi", out, "yol çokgeni", len(tris["road"]))
