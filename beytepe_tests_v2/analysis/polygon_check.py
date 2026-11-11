"""Lanelet poligon geçerlilik kontrolü (sol sınır + ters sağ sınır).
Ear-clipping triangülasyonunu bozan durumları arar: kendini kesen poligon, tekrar eden nokta,
sıfır alan, sınırların yer değiştirmesi (sol/sağ kesişmesi)."""
import json, sys
import numpy as np
import lanelet2
from lanelet2.projection import UtmProjector

MAP = sys.argv[1] if len(sys.argv) > 1 else "/autoware_map/lanelet2_map.osm"
m, _ = lanelet2.io.loadRobust(MAP, UtmProjector(lanelet2.io.Origin(39.87116440956932, 32.7356890084472)))
for p in m.pointLayer:
    a = p.attributes
    if "local_x" in a:
        p.x, p.y = float(a["local_x"]), float(a["local_y"])


def seg_intersections(P):
    """kapalı poligonda komşu olmayan kenar kesişimleri (vektörel)"""
    A = P; B = np.roll(P, -1, axis=0); n = len(P)
    hits = []
    for i in range(n):
        a, b = A[i], B[i]
        j = np.arange(i + 2, n)
        if i == 0:
            j = j[j != n - 1]
        if not len(j):
            continue
        c, d = A[j], B[j]
        def orient(p, q, r):
            return (q[..., 0] - p[..., 0]) * (r[..., 1] - p[..., 1]) - (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0])
        o1 = orient(a, b, c); o2 = orient(a, b, d); o3 = orient(c, d, a); o4 = orient(c, d, b)
        x = (o1 * o2 < 0) & (o3 * o4 < 0)
        for k in j[x]:
            hits.append((i, int(k)))
    return hits


out = []
for ll in m.laneletLayer:
    L = np.array([[q.x, q.y] for q in ll.leftBound]); R = np.array([[q.x, q.y] for q in ll.rightBound])
    P = np.vstack([L, R[::-1]])
    issues = []
    d = np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)
    if (d < 1e-3).any():
        issues.append(f"tekrar eden nokta x{int((d < 1e-3).sum())}")
    area = 0.5 * float(np.dot(P[:, 0], np.roll(P[:, 1], -1)) - np.dot(P[:, 1], np.roll(P[:, 0], -1)))
    if abs(area) < 0.5:
        issues.append(f"alan ~0 ({area:.2f} m²)")
    hits = seg_intersections(P)
    if hits:
        # kesişim yerini bul
        i, k = hits[0]
        issues.append(f"kendini kesiyor ({len(hits)} kesişim, ör. ~{P[i].round(1).tolist()})")
    sub = ll.attributes["subtype"] if "subtype" in ll.attributes else ""
    if issues:
        out.append(dict(id=ll.id, subtype=sub, n_left=len(L), n_right=len(R),
                        width_start=round(float(np.linalg.norm(L[0] - R[0])), 2),
                        width_end=round(float(np.linalg.norm(L[-1] - R[-1])), 2), issues=issues))
print(json.dumps(out, ensure_ascii=False))
