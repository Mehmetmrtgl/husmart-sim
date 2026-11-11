"""Bir (x, y) noktasının hangi lanelet içinde olduğunu ve en yakın lanelet'leri yazar.
Kullanım: python3 where_am_i.py <lanelet2_map.osm> x y [yaw_deg]"""
import sys, re, math
import lanelet2
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
osm = sys.argv[1]; x, y = float(sys.argv[2]), float(sys.argv[3])
txt = open(osm, encoding="utf-8").read()
m = re.search(r'lat="([-0-9.]+)"\s+lon="([-0-9.]+)"', txt)
mp, _ = lanelet2.io.loadRobust(osm, UtmProjector(lanelet2.io.Origin(float(m.group(1)), float(m.group(2)))))
for p in mp.pointLayer:
    if "local_x" in p.attributes:
        p.x = float(p.attributes["local_x"]); p.y = float(p.attributes["local_y"])
q = BasicPoint2d(x, y)
inside, near = [], []
for ll in mp.laneletLayer:
    if ll.attributes["subtype"] not in ("road", "road_shoulder"):
        continue
    d = lanelet2.geometry.distance(ll, q)
    if d == 0: inside.append(ll.id)
    near.append((d, ll.id))
near.sort()
print("nokta (%.2f, %.2f)" % (x, y))
print("içinde olduğu lanelet(ler):", inside or "YOK")
for d, i in near[:5]:
    ll = mp.laneletLayer[i]; c = ll.centerline
    dc = min(math.hypot(p.x - x, p.y - y) for p in c)
    print("  lanelet %d  kenara %.2f m  merkez çizgisine %.2f m  speed_limit %s  turn %s" % (
        i, d, dc, ll.attributes["speed_limit"] if "speed_limit" in ll.attributes else "-",
        ll.attributes["turn_direction"] if "turn_direction" in ll.attributes else "-"))
