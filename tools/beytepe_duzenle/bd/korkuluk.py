"""Guardrails and concrete parapets along the tour route (veri/korkuluklar.json -> scene objects).

The ends of every piece were read in the real data (real lidar map + ZED frames): where a rail starts and stops, where a
concrete parapet stands instead. The line itself follows the SIMULATED road edge (outline of the lanelet areas) at
`ofset_m` outside it, so the rail runs evenly along the road the car sees, also where the simulated road is a few metres
off the real one (north campus road). The scene builder keeps the posts off RoadRunner sidewalks / curbs and gives a
'gomulu' end a beam that slopes down into the ground, as on the real road.

geometri(A, H) -> dict(korkuluklar=[{ad, xy, uclar, not}], duvarlar=[{ad, tur, kalinlik, xy, boy}]) in the ROS map frame.
  rail polyline: the road is on its LEFT (the beam is mounted on the road side of the posts)
  wall polyline: the road-side face, the road is on its RIGHT (BeytepeDuzenleme.cs wall convention)
"""
import os
import numpy as np
from .ortak import yol, json_oku


def _halkalar(YOL):
    from shapely.geometry import LineString
    return [LineString(r.coords) for p in getattr(YOL, 'geoms', [YOL]) for r in [p.exterior, *p.interiors]]


def _yay(L, a, b):
    """part of the closed ring L from distance a forward to b (wrapping past the start)"""
    from shapely.ops import substring
    from shapely.geometry import LineString
    if b >= a:
        return substring(L, a, b)
    p1, p2 = substring(L, a, L.length), substring(L, 0, b)
    return LineString(list(p1.coords) + list(p2.coords)[1:])


def kenar(YOL, bas, son, ara=()):
    """road edge (lanelet area outline) from the projection of `bas` to that of `son`; `ara` points pick the side of the ring"""
    from shapely.geometry import Point
    rings = _halkalar(YOL)
    L = min(rings, key=lambda r: r.distance(Point(bas)) + r.distance(Point(son)) + sum(r.distance(Point(q)) for q in ara))
    a, b = L.project(Point(bas)), L.project(Point(son))
    ileri, geri = _yay(L, a, b), _yay(L, b, a)
    if ara:
        dI = sum(ileri.distance(Point(q)) for q in ara); dG = sum(geri.distance(Point(q)) for q in ara)
        sec = ileri if dI <= dG else geri
    else:
        sec = ileri if ileri.length <= geri.length else geri
    P = np.array(sec.coords)[:, :2]
    if sec is geri:
        P = P[::-1]
    return P, max(L.distance(Point(bas)), L.distance(Point(son)))


def _yeniden(P, adim):
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]; ss = np.linspace(0, s[-1], max(2, int(np.ceil(s[-1] / adim)) + 1))
    return np.c_[np.interp(ss, s, P[:, 0]), np.interp(ss, s, P[:, 1])]


def paralel(YOL, P, ofset):
    """line `ofset` m outside the road along edge P (1 m steps, normals from a 6 m tangent; pushed out where a concave corner
    of the outline brings it closer); returns the line and the side of the road ('sol' = road on the left of P)"""
    import shapely
    Q = _yeniden(P, 1.0); n = len(Q)
    T = np.array([Q[min(i + 3, n - 1)] - Q[max(i - 3, 0)] for i in range(n)]); T /= np.linalg.norm(T, axis=1)[:, None] + 1e-9
    Nl = np.c_[-T[:, 1], T[:, 0]]
    sol = shapely.contains(YOL, shapely.points(Q + Nl * 0.4)).sum() > shapely.contains(YOL, shapely.points(Q - Nl * 0.4)).sum()
    Nout = -Nl if sol else Nl
    R = Q + Nout * ofset
    for _ in range(3):           # concave corners: push the point out until it keeps the offset
        d = shapely.distance(YOL, shapely.points(R)); yakin = d < ofset - 0.15
        if not yakin.any():
            break
        R[yakin] += Nout[yakin] * (ofset - d[yakin])[:, None]
    # light smoothing (3 m running mean), ends kept
    S = R.copy(); S[1:-1] = (R[:-2] + R[1:-1] + R[2:]) / 3.0
    return S, ('sol' if sol else 'sag')


def geometri(A, H, sessiz=False):
    from shapely.geometry import LineString
    from .kenar import yol_alani
    p = yol(A['yollar'].get('korkuluklar', ''))
    if not p or not os.path.exists(p):
        if not sessiz:
            print('  korkuluk tanımı yok (%s), atlandı' % p)
        return None
    v = json_oku(p); YOL = yol_alani(H).buffer(0.05).buffer(-0.05)
    out = dict(korkuluklar=[], duvarlar=[])
    for o in v.get('korkuluklar', []) + v.get('beton_duvarlar', []):
        duvar = o in v.get('beton_duvarlar', [])
        P, uzak = kenar(YOL, o['bas'], o['son'], [tuple(q) for q in o.get('ara', [])])
        L_ = LineString(P).length; duz = np.hypot(*(np.array(o['son']) - np.array(o['bas'])))
        if uzak > 6.0 or L_ > 1.6 * duz + 30:
            raise SystemExit('HATA: korkuluk %s: yol kenarı bulunamadı (uç kenara %.1f m, kenar %.0f m / düz %.0f m)' % (o['ad'], uzak, L_, duz))
        R, yan = paralel(YOL, P, o.get('ofset_m', 1.3))
        uclar = list(o.get('uclar', ['gomulu', 'gomulu']))
        # rail: road on the left; wall: road on the right
        if (yan == 'sag') != duvar:
            R = R[::-1]; uclar = uclar[::-1]
        R = _yeniden(R, 2.0)
        ln = LineString(R)
        if ln.intersects(YOL):
            raise SystemExit('HATA: korkuluk %s şeritleri kesiyor' % o['ad'])
        xy = [round(float(c), 2) for q in R for c in q]
        if duvar:
            out['duvarlar'].append(dict(ad=o['ad'], tur='beton', kalinlik=o.get('kalinlik', 0.25), xy=xy, boy=[o.get('boy', 1.0)] * len(R)))
        else:
            out['korkuluklar'].append(dict(ad=o['ad'], xy=xy, uclar=uclar))
        if not sessiz:
            print('  %s %s: %.0f m, yol kenarından %.2f m%s' % ('beton duvar' if duvar else 'korkuluk', o['ad'], ln.length, ln.distance(YOL),
                                                               '' if duvar else ', uçlar ' + '/'.join(uclar)))
    if not sessiz:
        print('  korkuluk: %d parça %.0f m, beton duvar: %d parça %.0f m' % (
            len(out['korkuluklar']), sum(LineString(np.array(o['xy']).reshape(-1, 2)).length for o in out['korkuluklar']),
            len(out['duvarlar']), sum(LineString(np.array(o['xy']).reshape(-1, 2)).length for o in out['duvarlar'])))
    return out
