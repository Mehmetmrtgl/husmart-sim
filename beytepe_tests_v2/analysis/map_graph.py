"""v2 haritasının routing graph analizi (konteyner içinde çalışır).
Düğüm koordinatları local_x/local_y/ele etiketlerinden alınır (Autoware Local projektörüyle aynı).
Çıktı: JSON (stdout) — lanelet özeti, bağlı bileşenler, kuzey yola erişim."""
import json, math, sys
import numpy as np
import lanelet2
from lanelet2.projection import UtmProjector
from autoware_lanelet2_extension_python.utility import utilities  # noqa: F401 (kayıt için)

def aget(a, k, d=""):
    return a[k] if k in a else d


MAP = sys.argv[1] if len(sys.argv) > 1 else "/autoware_map/lanelet2_map.osm"
proj = UtmProjector(lanelet2.io.Origin(39.87116440956932, 32.7356890084472))
m, errs = lanelet2.io.loadRobust(MAP, proj)

for p in m.pointLayer:
    a = p.attributes
    if "local_x" in a:
        p.x, p.y = float(a["local_x"]), float(a["local_y"])  # z: lanelet2 ele etiketinden zaten okur

rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                      lanelet2.traffic_rules.Participants.Vehicle)
road = [ll for ll in m.laneletLayer if aget(ll.attributes, "subtype") == "road"]
graph = lanelet2.routing.RoutingGraph(m, rules)

def cl(ll):
    return np.array([[p.x, p.y, p.z] for p in ll.centerline])

info = {}
for ll in road:
    c = cl(ll)
    seg = np.diff(c[:, :2], axis=0)
    length = float(np.linalg.norm(seg, axis=1).sum())
    hd = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0])) if len(seg) else np.zeros(1)
    info[ll.id] = dict(
        id=ll.id, length=round(length, 1),
        heading_change_deg=round(float(np.degrees(abs(hd[-1] - hd[0]))), 1),
        turn=aget(ll.attributes, "turn_direction"),
        start=c[0].round(2).tolist(), end=c[-1].round(2).tolist(),
        mid=c[len(c) // 2].round(2).tolist(),
        succ=[s.id for s in graph.following(ll)],
        left=[x.id for x in [graph.left(ll)] if x], right=[x.id for x in [graph.right(ll)] if x],
        y_mid=float(c[len(c) // 2][1]),
    )

# bileşenler (yalnız ileri + şerit değişimi kenarları ile erişilebilirlik)
ids = list(info)
reach = {}
for i in ids:
    ll = m.laneletLayer[i]
    reach[i] = {x.id for x in graph.reachableSet(ll, 1e9, 0, True)}

north = [i for i in ids if info[i]["y_mid"] > 1000]
south = [i for i in ids if info[i]["y_mid"] < 300]
s2n = [i for i in south if any(n in reach[i] for n in north)]
n2s = [i for i in north if any(s in reach[i] for s in south)]
dead_ends = [i for i in ids if not info[i]["succ"]]
no_pred = [i for i in ids if not graph.previous(m.laneletLayer[i])]

out = dict(
    load_errors=len(errs), n_road=len(road), n_north=len(north), n_south=len(south),
    south_lanelets_reaching_north=len(s2n), north_lanelets_reaching_south=len(n2s),
    dead_ends=dead_ends, no_predecessor=no_pred,
    avg_reach=round(float(np.mean([len(reach[i]) for i in ids])), 1),
    lanelets=info,
)
print(json.dumps(out))
