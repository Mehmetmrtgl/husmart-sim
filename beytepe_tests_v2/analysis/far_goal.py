"""Verilen lanelet'ten rota ile ulaşılabilen lanelet'leri bulur, rota uzunluğuna göre en uzakları listeler.
Kullanım (Autoware imajında): python3 far_goal.py <lanelet2_map.osm> <başlangıç_lanelet_id> [adet]
"""
import sys, math, re
import lanelet2
from lanelet2.projection import UtmProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import create as tr_create, Locations, Participants

osm, start_id = sys.argv[1], int(sys.argv[2])
top = int(sys.argv[3]) if len(sys.argv) > 3 else 10
txt = open(osm, encoding="utf-8").read()
m = re.search(r'lat="([-0-9.]+)"\s+lon="([-0-9.]+)"', txt)
proj = UtmProjector(lanelet2.io.Origin(float(m.group(1)), float(m.group(2))))
mp, _ = lanelet2.io.loadRobust(osm, proj)
# harita çerçevesi: local_x/local_y etiketleri
for p in mp.pointLayer:
    if "local_x" in p.attributes and "local_y" in p.attributes:
        p.x = float(p.attributes["local_x"]); p.y = float(p.attributes["local_y"])
rules = tr_create(Locations.Germany, Participants.Vehicle)
g = RoutingGraph(mp, rules)
start = mp.laneletLayer[start_id]
L = lambda ll: lanelet2.geometry.length2d(ll)
res = []
for ll in g.reachableSet(start, 1e9, 0, True):
    if ll.id == start.id: continue
    r = g.getRoute(start, ll, 0, True)
    if r is None: continue
    sp = r.shortestPath()
    d = sum(L(x) for x in sp)
    c = ll.centerline
    res.append((d, ll.id, len(sp), c[len(c) // 2].x, c[len(c) // 2].y, L(ll)))
res.sort(reverse=True)
s = start.centerline
print("başlangıç %d (%.1f, %.1f), ulaşılabilir lanelet: %d" % (start.id, s[0].x, s[0].y, len(res)))
for d, i, n, x, y, ln in res[:top]:
    print("hedef %d  rota %.0f m  (%d lanelet)  orta nokta (%.1f, %.1f)  uzunluk %.0f m" % (i, d, n, x, y, ln))
