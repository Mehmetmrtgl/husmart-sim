"""Trees along the tour route, from the real lidar map (stage 3 of the route work).

kenar-tespit: the real drive (gercek_poz) covers the whole tour route. Around it (yol_kenari.koridor_m) every trunk with a
crown is measured: trunk slice 0.7-2.3 m above the ground, a narrow cluster (<= 1.8 m), a top of at least tac_min_m and a
crown spread of at least yayilim_min_m (poles, signs, people and parked cars have no crown) -> veri/yol_kenari_nesneler.json.

awsim (geometri): trees on a lane, a median / gate island or in the gate area are dropped (those places have their own
trees); the corridor is cut into pieces without holes. In AWSIM the RoadRunner trees inside the pieces are replaced by
the measured ones, and the pieces are part of the point-cloud update region.
"""
import os
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from .ortak import Harita, yol, json_oku, json_yaz, pcd_oku


def gercek_iz(A, adim=5.0):
    """Real drive path (x, y) resampled every `adim` metres."""
    X = np.load(yol(A['yollar']['gercek_poz']))['X'][:, :2, 3]
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(X, axis=0).T))]; ss = np.arange(0, s[-1], adim)
    return np.c_[np.interp(ss, s, X[:, 0]), np.interp(ss, s, X[:, 1])]


def yol_alani(H):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    ps = [Polygon(H.serit_alani(l)).buffer(0) for l, (m, t) in H.R.items() if t.get('type') == 'lanelet' and 'left' in m]
    return unary_union([p for p in ps if p.area > 0.5])


def zemin(p, hucre=1.0):
    """Ground height per cell: 10th percentile, cells more than 0.4 m above the 5x5 minimum are replaced by it, holes filled."""
    x0, y0 = np.floor(p[:, 0].min()), np.floor(p[:, 1].min())
    ix = ((p[:, 0] - x0) / hucre).astype(int); iy = ((p[:, 1] - y0) / hucre).astype(int); nx, ny = ix.max() + 1, iy.max() + 1
    key = iy * nx + ix; o = np.lexsort((p[:, 2], key)); ks = key[o]; zs = p[o, 2]
    u, st, cnt = np.unique(ks, return_index=True, return_counts=True)
    G = np.full(nx * ny, np.nan); ok = cnt >= 3; G[u[ok]] = zs[st[ok] + (cnt[ok] * 0.1).astype(int)]; G = G.reshape(ny, nx)
    Gf = np.where(np.isnan(G), 1e9, G); Gm = ndimage.minimum_filter(Gf, size=5)
    G = np.where(np.isnan(G) | (G - Gm > 0.4), Gm, G); G[G > 1e8] = np.nan
    idx = ndimage.distance_transform_edt(np.isnan(G), return_distances=False, return_indices=True); G = ndimage.median_filter(G[idx[0], idx[1]], size=3)
    return p[:, 2] - G[iy, ix]


def tespit(A):
    import shapely
    K = A['yol_kenari']
    H = Harita(yol(A['yollar']['harita'])); YOL = yol_alani(H)
    iz = gercek_iz(A)
    p = pcd_oku(yol(A['yollar']['gercek_lidar']), onbellek=os.path.join(yol(A['yollar']['ara']), 'gercek_bulut.npy'))[:, :3]
    d, _ = cKDTree(iz).query(p[:, :2], distance_upper_bound=K['koridor_m'] + 4)
    p = p[np.isfinite(d)]; h = zemin(p)
    print(f'  gerçek iz {len(iz) * 5} m, koridor ({K["koridor_m"]} m) noktası {len(p)}')
    g0, g1 = K['govde_h']
    m = (h > g0) & (h < g1)
    m[m] = ~shapely.contains(YOL.buffer(K['yol_payi_m']), shapely.points(p[m, 0], p[m, 1]))
    q = p[m]; r = 0.25; x0, y0 = q[:, 0].min() - 1, q[:, 1].min() - 1
    gi = ((q[:, 1] - y0) / r).astype(int); gj = ((q[:, 0] - x0) / r).astype(int)
    Gs = np.zeros((gi.max() + 2, gj.max() + 2), np.int32); np.add.at(Gs, (gi, gj), 1)
    lab, _ = ndimage.label(Gs > 0, structure=np.ones((3, 3)))
    tk = cKDTree(p[:, :2]); aday = []
    for c, sl in enumerate(ndimage.find_objects(lab), 1):
        sub = lab[sl] == c; w = Gs[sl] * sub
        if w.sum() < 5 or max(sub.shape) * r > 1.8:
            continue
        ii, jj = np.nonzero(sub)
        cy = (sl[0].start + np.average(ii, weights=w[ii, jj]) + .5) * r + y0; cx = (sl[1].start + np.average(jj, weights=w[ii, jj]) + .5) * r + x0
        idx = tk.query_ball_point([cx, cy], 4.0); rr = np.hypot(p[idx, 0] - cx, p[idx, 1] - cy); hh = h[idx]
        core = rr < 0.5
        if core.sum() < 5:
            continue
        top = float(np.percentile(hh[core], 99))
        up = (hh > max(2.5, 0.45 * top)) & (hh < top + 0.5)
        spread = float(np.percentile(rr[up], 90)) if up.sum() > 10 else 0.0
        if top >= K['tac_min_m'] and spread >= K['yayilim_min_m']:
            aday.append((int(w.sum()), cx, cy, top, spread))
    agaclar = []                       # strongest trunk first; a second trunk within 1 m of a tree is the same tree
    for n, cx, cy, top, spread in sorted(aday, reverse=True):
        if any(np.hypot(cx - t['xy'][0], cy - t['xy'][1]) < 1.0 for t in agaclar):
            continue
        boy = round(min(top, K['boy_max_m']), 1)
        agaclar.append(dict(xy=[round(cx, 2), round(cy, 2)], boy=boy, tac=round(float(np.clip(spread, 0.2 * boy, 0.5 * boy)), 2)))
    agaclar.sort(key=lambda t: (t['xy'][1], t['xy'][0]))
    for i, t in enumerate(agaclar):
        t['ad'] = 'K%04d' % (i + 1)
    b = np.array([t['boy'] for t in agaclar])
    print(f'  ağaç {len(agaclar)} (boy medyan {np.median(b):.1f} m, 10 m üstü {(b >= 10).sum()})')
    return dict(aciklama='Trees beside the tour route, measured in the real lidar map (duzenle.py kenar-tespit). ROS map frame.',
                koridor_m=K['koridor_m'], agaclar=agaclar)


def tespit_calistir(A, hedef=None):
    print('kenar-tespit')
    veri = tespit(A); hedef = hedef or yol(A['yollar']['yol_kenari_nesneler'])
    json_yaz(hedef, veri, A if hedef == yol(A['yollar']['yol_kenari_nesneler']) else None)
    print('  ->', hedef)
    return veri


def geometri(A, H, adalar, kutu):
    """Scene data: trees off the lanes, the islands (`adalar`, shapely) and the gate box (`kutu`), corridor pieces."""
    from shapely.geometry import Point, LineString, Polygon
    K = A['yol_kenari']; p_ = yol(A['yollar']['yol_kenari_nesneler'])
    if not os.path.exists(p_):
        print('  yol kenarı: %s yok ("kenar-tespit" çalıştırılmadı), atlandı' % p_); return None
    v = json_oku(p_); YOL = yol_alani(H)
    yasak = YOL.buffer(K['yol_payi_m']).union(adalar.buffer(K['ada_payi_m'])).union(kutu)
    nodes = np.array(list(H.N.values())); tn = cKDTree(nodes[:, :2])
    agaclar = []; n_at = 0
    for t in v['agaclar']:
        x, y = t['xy']
        if yasak.contains(Point(x, y)):
            n_at += 1; continue
        h = float(t['boy'])
        agaclar.append(dict(tur='agac' if h >= 4.5 else 'cali', x=x, y=y, z=round(float(nodes[tn.query([x, y])[1], 2]), 2), boy=h, tac=t['tac']))
    # corridor pieces along the real drive, ~150 m each (a piece of a closed loop would otherwise get a hole)
    iz = gercek_iz(A); n = int(K['parca_m'] / 5); parcalar = []
    for i in range(0, len(iz) - 1, n):
        g = LineString(iz[i:i + n + 1]).buffer(K['koridor_m'], resolution=4).simplify(0.5)
        g = Polygon(g.exterior)                              # fill any hole (sharp turns)
        parcalar.append(dict(xy=[round(float(c), 2) for q in np.array(g.exterior.coords)[:-1] for c in q]))
    from shapely.ops import unary_union
    alan = unary_union([Polygon(np.array(a['xy']).reshape(-1, 2)) for a in parcalar]).area
    print('  yol kenarı: %d ağaç + %d çalı (şerit/ada/nizamiye üstündeki %d atlandı), %d koridor parçası %.0f m2'
          % (sum(t['tur'] == 'agac' for t in agaclar), sum(t['tur'] == 'cali' for t in agaclar), n_at, len(parcalar), alan))
    return dict(agaclar=agaclar, gizle=parcalar)
