"""Median island (refüj) between the two carriageways, from the campus entrance up to the gate U-turn.

refuj-tespit (needs the real-vehicle data, not in the repo):
    real lidar map + lanelet edges -> lamp and tree positions -> veri/refuj_nesneler.json
    lamps: head points 7-11.5 m above ground spanning > 4 m across the median, then the regular ~45 m spacing
           fills the gaps; lamps seen only in the ZED camera are added from ayarlar.yaml (refuj.kamera_lamba_y).
    trees: trunk clusters 0.7-2.3 m above ground with a crown >= 2.5 m.
awsim (geometri): edge ring from the lanelet borders -> curbs (kerb width / height), grass mound, curb faces,
    lamps and trees with heights, and the point-cloud sampling regions.
"""
import os
import numpy as np
from scipy.spatial import cKDTree
from scipy import ndimage
from .ortak import Harita, yol, json_oku, json_yaz, pcd_oku


# ---------- shared ----------
def ucgenle(geom, V, T, zf, hucre=1.0):
    """Triangulate a shapely (Multi)Polygon in hucre x hucre pieces (keeps the exact outline, gives small triangles).
    Appends to V (xyz list) and T (index list); returns the number of triangles."""
    from shapely.geometry import box
    from shapely import constrained_delaunay_triangles
    minx, miny, maxx, maxy = geom.bounds; n = 0
    for x0 in np.arange(np.floor(minx), maxx, hucre):
        for y0 in np.arange(np.floor(miny), maxy, hucre):
            pc = geom.intersection(box(x0, y0, x0 + hucre, y0 + hucre))
            if pc.is_empty or pc.area < 1e-4:
                continue
            for g in getattr(pc, 'geoms', [pc]):
                if g.geom_type != 'Polygon' or g.area < 1e-4:
                    continue
                for tr in constrained_delaunay_triangles(g).geoms:
                    c = np.array(tr.exterior.coords)[:3]; b = len(V)
                    for x, y in c:
                        V.append([x, y, zf(x, y)])
                    T.extend([b, b + 1, b + 2]); n += 1
    return n


def kenarlar(A, H, plan):
    """Inner borders of the two carriageways. east: northbound, south -> north; west: southbound, north -> south;
    uin: inner border of the U-turn (east -> west)."""
    Rf, R = A['refuj'], plan['R']
    sol = lambda lid: H.P(H.R[lid][0]['left'])
    dogu = [sol(l) for l in Rf['dogu_seritler']] + [sol(R['A1']), sol(R['A2'])]
    east = np.concatenate([dogu[0]] + [d[1:] for d in dogu[1:]])
    west = np.r_[sol(R['G1c']), sol(R['G1d'])[1:], sol(Rf['bati_serit'])[1:]]
    for lid in Rf.get('bati_seritler_ek', []):
        west = np.r_[west, sol(lid)[1:]]
    uin = sol(R['U'])[1:-1]
    return east, west, uin


def eksen(A, east, west, y0=None):
    """Median axis: midpoints of the two inner borders north of y0 (default eksen_baslangic_y), resampled every 1 m and smoothed.
    Returns ax (n,2), unit tangent t (south -> north) and left normal n (towards the west border)."""
    from scipy.ndimage import uniform_filter1d
    e2, w2 = east[:, :2], west[:, :2]; tw = cKDTree(w2); mids = []
    for e in e2:
        if e[1] < (A['refuj']['eksen_baslangic_y'] if y0 is None else y0):
            continue
        d, j = tw.query(e)
        if d < 20:
            mids.append((e + w2[j]) / 2)
    mids = np.array(mids); s = np.r_[0, np.cumsum(np.hypot(*np.diff(mids, axis=0).T))]
    ss = np.arange(0, s[-1], 1.0); ax = np.c_[np.interp(ss, s, mids[:, 0]), np.interp(ss, s, mids[:, 1])]
    ax = np.c_[uniform_filter1d(ax[:, 0], 9, mode='nearest'), uniform_filter1d(ax[:, 1], 9, mode='nearest')]
    t = np.gradient(ax, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]
    return ax, t, np.c_[-t[:, 1], t[:, 0]]


# ---------- refuj-tespit ----------
def tespit(A):
    Rf = A['refuj']
    H = Harita(yol(A['yollar']['harita'])); plan = json_oku(os.path.join(yol(A['yollar']['ara']), 'nizamiye_plan.json'))
    # the median is one island from y ~300, but its south part has its own detection (refuj-guney-tespit, up to guney.y_son):
    # this one starts where that one ends, so the two object files never overlap
    east, west, _ = kenarlar(A, H, plan); ax, tv, nv = eksen(A, east, west, y0=Rf['guney']['y_son']); S_ = len(ax); tax = cKDTree(ax)

    def sn(P):       # (along, across) coordinates relative to the axis
        _, i = tax.query(P); v = P - ax[i]; return i + (v * tv[i]).sum(1), (v * nv[i]).sum(1)
    # real lidar around the median
    lid = yol(A['yollar']['gercek_lidar'])
    if not os.path.exists(lid):
        raise SystemExit('HATA: gerçek lidar haritası yok: ' + lid)
    p = pcd_oku(lid)
    lo, hi = ax.min(0) - 15, ax.max(0) + 15
    p = p[(p[:, 0] > lo[0]) & (p[:, 0] < hi[0]) & (p[:, 1] > lo[1]) & (p[:, 1] < hi[1]), :3]
    qs, qn = sn(p[:, :2]); keep = (abs(qn) < 12) & (qs >= 0) & (qs < S_); q, qs, qn = p[keep], qs[keep], qn[keep]
    print(f'  eksen {S_} m, lidar noktası {len(q)}')
    # ground: 10th percentile per (1 m along, 0.5 m across) cell, then the minimum over 5 x 3.5 m, holes filled
    G = np.full((S_, 48), np.nan); is_ = np.clip(qs.astype(int), 0, S_ - 1); in_ = np.clip(((qn + 12) / 0.5).astype(int), 0, 47)
    key = is_ * 48 + in_; o = np.argsort(key); ks = key[o]; z = q[o, 2]; u, st = np.unique(ks, return_index=True); en = np.r_[st[1:], len(ks)]
    for k, a, b in zip(u, st, en):
        if b - a >= 3:
            G.flat[k] = np.percentile(z[a:b], 10)
    Gf = G.copy(); Gf[np.isnan(Gf)] = 1e9; Gm = ndimage.minimum_filter(Gf, size=(5, 7)); Gm[Gm > 1e8] = np.nan
    idx = ndimage.distance_transform_edt(np.isnan(Gm), return_distances=False, return_indices=True); Gm = Gm[idx[0], idx[1]]
    hag = q[:, 2] - Gm[is_, in_]
    se, ne = sn(east[:, :2]); sw, nw = sn(west[:, :2])
    he = np.interp(np.arange(S_), se, -ne); hw = np.interp(np.arange(S_), sw, nw)
    H_E = np.interp(qs, np.arange(S_), he); H_W = np.interp(qs, np.arange(S_), hw)
    inmed = (qn > -H_E + 0.3) & (qn < H_W - 0.3)

    # lamp heads: thin peaks of the along-axis histogram whose points spread > 4 m across (the two arms)
    from scipy.signal import find_peaks
    mh = (hag > 7) & (hag < 11.5) & (qn > -H_E - 1) & (qn < H_W + 1)
    sb = np.arange(0, S_ + 0.5, 0.5); cnt, _ = np.histogram(qs[mh], sb); nspan = np.zeros(len(cnt))
    for i in range(len(cnt)):
        m = mh & (qs >= sb[i]) & (qs < sb[i + 1])
        if m.sum() >= 3:
            nspan[i] = np.percentile(qn[m], 95) - np.percentile(qn[m], 5)
    bg = ndimage.uniform_filter1d(cnt.astype(float), 13)
    score = np.where(nspan > 4.0, cnt / (bg + 1.0), 0)
    pk, _ = find_peaks(score, height=1.8, distance=30)
    s_ad = np.array([sb[i] + 0.25 for i in pk]); puan = score[pk]
    def xy(s_, n):   # inverse of sn()
        i = int(np.clip(round(s_), 0, S_ - 1)); return ax[i] + tv[i] * (s_ - i) + nv[i] * n
    # trunks: compact clusters 0.7-2.3 m above ground inside the median
    ms = (hag > 0.7) & (hag < 2.3) & inmed; r = 0.25
    gi = (qs[ms] / r).astype(int); gj = ((qn[ms] + 12) / r).astype(int)
    Gs = np.zeros((int(S_ / r) + 2, int(24 / r) + 2), int); np.add.at(Gs, (gi, gj), 1)
    lab, _ = ndimage.label(Gs > 0, structure=np.ones((3, 3))); govde = []
    for k, sl in enumerate(ndimage.find_objects(lab), 1):
        sub = (lab[sl] == k); w = Gs[sl] * sub; c_ = w.sum(); ext = max(sub.shape) * r
        if c_ < 5 or ext > 1.8:
            continue
        ii, jj = np.nonzero(sub)
        s0 = (sl[0].start + np.average(ii, weights=w[ii, jj]) + .5) * r; n0 = (sl[1].start + np.average(jj, weights=w[ii, jj]) + .5) * r - 12
        near = (abs(qs - s0) < 0.5) & (abs(qn - n0) < 0.5); top = float(np.percentile(hag[near], 99)) if near.sum() > 3 else 0
        govde.append(dict(s=float(s0), xy=xy(s0, n0).tolist(), nokta=int(c_), tepe=round(top, 1)))

    # lamp row: strong lidar lamps + confirmed lamps are anchors; gaps are filled at the regular spacing,
    # snapping to a weak lidar candidate within 5 m when there is one
    s_of = lambda yy: float(np.argmin(abs(ax[:, 1] - yy)))
    kaynak = {a: 'lidar (güçlü)' for a in s_ad[puan >= 3.5]}
    for yy in Rf['sabit_lamba_y']:
        kaynak[s_ad[np.argmin(abs(np.array([xy(s, 0)[1] for s in s_ad]) - yy))]] = 'lidar + kamera'
    for yy in Rf['kamera_lamba_y']:
        kaynak[s_of(yy)] = 'kamera'
    capa = sorted(kaynak); out = []
    for a, b in zip(capa[:-1], capa[1:]):
        k = max(1, int(round((b - a) / Rf['lamba_aralik']))); out.append((a, kaynak[a]))
        for j in range(1, k):
            se_ = a + (b - a) * j / k; near = [si for si in s_ad if abs(si - se_) < 5]
            out.append((min(near, key=lambda v: abs(v - se_)), 'lidar (zayıf, dizide)') if near else (se_, 'dizi (eşit aralık)'))
    out.append((capa[-1], kaynak[capa[-1]]))
    lambalar = [dict(xy=xy(s, 0).tolist(), kaynak=src, ad='L%02d' % (i + 1)) for i, (s, src) in enumerate(out)]
    # trees: trunks with a crown >= 2.5 m, not on a lamp, merged within 1.5 m
    agaclar = []
    for t in sorted([g for g in govde if g['tepe'] >= 2.5], key=lambda a: -a['nokta']):
        p_ = np.array(t['xy'])
        if any(np.hypot(*(p_ - np.array(l['xy']))) < 1.5 for l in lambalar) or any(np.hypot(*(p_ - np.array(u['xy']))) < 1.5 for u in agaclar):
            continue
        agaclar.append(dict(xy=p_.tolist(), s=t['s'], boy=round(max(t['tepe'], 4.0), 1), buyuk=t['tepe'] >= 6))
    agaclar.sort(key=lambda a: a['s'])
    for i, t in enumerate(agaclar):
        t['ad'] = 'A%03d' % (i + 1)
    print(f'  lamba {len(lambalar)}, ağaç {len(agaclar)} (büyük {sum(t["buyuk"] for t in agaclar)})')
    return dict(lambalar=lambalar, agaclar=agaclar)


def tespit_calistir(A, hedef=None):
    print('refuj-tespit')
    veri = tespit(A); hedef = hedef or yol(A['yollar']['refuj_nesneler'])
    json_yaz(hedef, veri, A if hedef == yol(A['yollar']['refuj_nesneler']) else None)
    print('  ->', hedef)
    return veri


# ---------- AWSIM geometry ----------
def halka(xy, z_fn=None, z=None, adim=0.5):
    """Closed ring resampled every adim m; z from z_fn(x, y) or interpolated from z (per input point)."""
    xy = np.asarray(xy, float)[:, :2]
    if np.hypot(*(xy[0] - xy[-1])) > 1e-6:
        xy = np.r_[xy, xy[:1]]
        if z is not None:
            z = np.r_[z, z[:1]]
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]; ss = np.arange(0, s[-1], adim)
    out = np.c_[np.interp(ss, s, xy[:, 0]), np.interp(ss, s, xy[:, 1])]
    zz = np.interp(ss, s, z) if z is not None else np.array([z_fn(x, y) for x, y in out])
    return np.c_[out, zz]


def kaldirimli_ada(loop, KG, KH, TUM, pay=0.0, asfalt_maske=None):
    """Raised island inside a ring at the road edge (loop: x, y, road height; resampled ~0.5 m):
    a curb strip KG wide and KH high, grass inside (meets the curb tops, +TUM mound 1.5 m away from the curbs),
    a vertical curb face. With pay > 0 the curb stands pay m inside the ring and the strip in between
    (from 8 cm, so the edge line stays visible) is asphalt at road height - only inside asfalt_maske when given
    (along the lanes; around the median nose there is no road, the strip lay on the grass as grey shelves)."""
    from shapely.geometry import Polygon
    dis = Polygon(loop[:, :2]).buffer(0)
    t0 = cKDTree(loop[:, :2]); n0 = len(loop)

    def zyol(x, y, L=loop, t=t0, n=n0):   # road height at the ring: closest point of the polyline, interpolated
        i = t.query([x, y])[1]; p = np.array([x, y]); best = (1e9, L[i, 2])
        for j in (i - 1, i):
            a, b = L[j % n], L[(j + 1) % n]; ab = b[:2] - a[:2]
            tt = np.clip(np.dot(p - a[:2], ab) / max(np.dot(ab, ab), 1e-9), 0, 1); d = np.hypot(*(a[:2] + tt * ab - p))
            if d < best[0]:
                best = (d, a[2] + tt * (b[2] - a[2]))
        return best[1]
    poly = dis
    if pay > 0:
        poly = dis.buffer(-pay, join_style=2)
        if poly.geom_type != 'Polygon':
            poly = max(poly.geoms, key=lambda g: g.area)
        kenar = halka(np.array(poly.exterior.coords), z_fn=zyol)
    else:
        kenar = loop
    inner = poly.buffer(-KG, join_style=2)
    tl = cKDTree(kenar[:, :2]); nl = len(kenar); K = min(200, nl)

    def zcim(x, y):
        d, i = tl.query([x, y], k=K)
        w = 1.0 / (np.maximum(d - KG, 0.0) + 0.02) ** 2
        u = np.clip((d[0] - KG) / 1.5, 0, 1)
        return float((kenar[i, 2] * w).sum() / w.sum() + KH + TUM * u * u * (3 - 2 * u))
    zk = lambda x, y: zyol(x, y, kenar, tl, nl)
    KV, KT, CV, CT, SV, ST, BV, BT = [], [], [], [], [], [], [], []
    ucgenle(poly.difference(inner) if not inner.is_empty else poly, KV, KT, lambda x, y: zk(x, y) + KH)
    if not inner.is_empty:
        ucgenle(inner, CV, CT, zcim)
    if pay > 0:
        serit = dis.buffer(-0.08, join_style=2).difference(poly)
        if asfalt_maske is not None:
            serit = serit.intersection(asfalt_maske)
        ucgenle(serit, SV, ST, lambda x, y: zyol(x, y) + 0.005)
    for k in range(nl):                                           # curb face: vertical strip along the curb edge
        a = kenar[k]; b = kenar[(k + 1) % nl]; i0 = len(BV)
        BV += [[a[0], a[1], a[2] - 0.05], [a[0], a[1], a[2] + KH], [b[0], b[1], b[2] - 0.05], [b[0], b[1], b[2] + KH]]
        BT += [i0, i0 + 2, i0 + 1, i0 + 1, i0 + 2, i0 + 3]
    return dict(dis=dis, ic=poly, cim=inner, zcim=zcim, KV=KV, KT=KT, CV=CV, CT=CT, SV=SV, ST=ST, BV=BV, BT=BT)


def geometri(A, H, plan, nesneler=None):
    from shapely.geometry import Polygon, Point, box
    Rf = A['refuj']; KG, KH, TUM = Rf['kaldirim_genislik'], Rf['kaldirim_yukseklik'], Rf['tumsek']
    nesneler = nesneler or json_oku(yol(A['yollar']['refuj_nesneler']))
    east, west, uin = kenarlar(A, H, plan); ax, tv, nv = eksen(A, east, west)
    Y0 = ax[0, 1]                                                  # south tip of the median
    east = east[east[:, 1] >= Y0 - 0.5]; west = west[west[:, 1] >= Y0 - 0.5]
    # south nose: half circle from the end of the west border to the start of the east border, bulging south
    c0 = (east[0, :2] + west[-1, :2]) / 2; rr = np.hypot(*(east[0, :2] - west[-1, :2])) / 2
    a0 = np.arctan2(*(west[-1, :2] - c0)[::-1]); a1 = np.arctan2(*(east[0, :2] - c0)[::-1])
    geri = -tv[0]
    a1 = max([a1, a1 - 2 * np.pi if a1 > a0 else a1 + 2 * np.pi], key=lambda b: np.dot([np.cos((a0 + b) / 2), np.sin((a0 + b) / 2)], geri))
    th = np.linspace(a0, a1, 24)[1:-1]
    nose = np.c_[c0[0] + rr * np.cos(th), c0[1] + rr * np.sin(th), np.linspace(west[-1, 2], east[0, 2], len(th))]
    loop = np.r_[east, uin, west, nose, east[:1]]
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(loop[:, :2], axis=0).T))]; ss = np.arange(0, s[-1], 0.5)
    loop = np.c_[[np.interp(ss, s, loop[:, k]) for k in range(3)]].T
    # the road height along the ring, smoothed over ~5 m (lanelet node heights are a few cm noisy: the curbs looked wavy)
    from scipy.ndimage import uniform_filter1d
    loop[:, 2] = uniform_filter1d(loop[:, 2], 11, mode='wrap')
    # the south nose is not at a road: between the two carriageways RoadRunner's grass bulges up to ~0.3 m above the
    # straight line between their heights, and stood over the curb there (2026-11-24). Its ring follows the ground of the
    # base cloud (30th percentile within 0.6 m), never lower than before.
    ara = yol(A['yollar']['ara']); B_ = pcd_oku(yol(A['yollar']['temel_bulut']), onbellek=os.path.join(ara, 'temel_bulut.npy'))[:, :3]
    lo_, hi_ = nose[:, :2].min(0) - 3, nose[:, :2].max(0) + 3
    B_ = B_[(B_[:, 0] > lo_[0]) & (B_[:, 0] < hi_[0]) & (B_[:, 1] > lo_[1]) & (B_[:, 1] < hi_[1])]
    if len(B_):
        tb_, tn_ = cKDTree(B_[:, :2]), cKDTree(nose[:, :2])
        burun = np.flatnonzero(tn_.query(loop[:, :2])[0] < 0.6); n_up = 0
        for i in burun:
            dis_ = loop[i, :2] - c0; dis_ /= max(np.hypot(*dis_), 1e-6)      # outward (away from the nose centre)
            zs_ = []
            for off in (0.0, 0.5):                                          # on the ring and 0.5 m outside it
                ii = tb_.query_ball_point(loop[i, :2] + off * dis_, 0.5)
                if len(ii) >= 3:
                    zs_.append(float(np.percentile(B_[ii, 2], 30)))
            if zs_ and max(zs_) - 0.02 > loop[i, 2]:
                loop[i, 2] = max(zs_) - 0.02; n_up += 1
        print('  güney burnu: %d / %d halka noktası zemine kaldırıldı' % (n_up, len(burun)))
    # cut the median where a crosswalk crosses it (the real median is open there); every piece is its own island
    tl0 = cKDTree(loop[:, :2]); n0 = len(loop)

    def zring(x, y):          # road height at the ring, interpolated on the closest segment
        i = tl0.query([x, y])[1]; p_ = np.array([x, y]); best = (1e9, loop[i, 2])
        for j in (i - 1, i):
            a, b = loop[j % n0], loop[(j + 1) % n0]; ab = b[:2] - a[:2]
            tt = np.clip(np.dot(p_ - a[:2], ab) / max(np.dot(ab, ab), 1e-9), 0, 1); d = np.hypot(*(a[:2] + tt * ab - p_))
            if d < best[0]:
                best = (d, a[2] + tt * (b[2] - a[2]))
        return best[1]
    from shapely.ops import unary_union
    dis0 = Polygon(loop[:, :2]).buffer(0)
    gecit = [Polygon(np.r_[H.P(mm['left'])[:, :2], H.P(mm['right'])[::-1, :2]]).buffer(0) for lid, (mm, t) in H.R.items()
             if t.get('type') == 'lanelet' and t.get('subtype') == 'crosswalk' and 'left' in mm]
    gecit = [c for c in gecit if c.area > 0 and c.intersects(dis0)]
    parcalar = [dis0]
    if gecit:
        kes = dis0.difference(unary_union(gecit).buffer(Rf.get('yaya_gecidi_boslugu_m', 0.5)))
        parcalar = sorted([q for q in getattr(kes, 'geoms', [kes]) if q.geom_type == 'Polygon' and q.area > 20], key=lambda q: -q.area)
        if len(parcalar) < 2:       # the crosswalk ends at the median edge (does not cross it): no notch, the median stays whole
            print('  yaya geçidi refüjü boydan boya kesmiyor, refüj bütün kaldı'); parcalar = [dis0]
        else:
            print('  refüj %d yaya geçidinde açıldı -> %d parça' % (len(gecit), len(parcalar)))
    adalar = []
    from shapely.geometry import LineString
    pay_ = Rf.get('kenar_payi_m', 0.0)
    maske = unary_union([LineString(b[:, :2]).buffer(pay_ + 0.2, cap_style=2) for b in (east, west, uin) if len(b) > 1])
    for q in parcalar:
        lp = loop if q is dis0 else halka(np.array(q.exterior.coords), z_fn=zring)
        adalar.append((lp, kaldirimli_ada(lp, KG, KH, TUM, pay_, maske)))
    loop, g = adalar[0]
    poly = unary_union([g_['dis'] for _, g_ in adalar]); ta = cKDTree(ax)

    def zcim(x, y):
        for _, g_ in adalar:
            if g_['dis'].buffer(0.5).contains(Point(x, y)):
                return g_['zcim'](x, y)
        return adalar[0][1]['zcim'](x, y)
    print('  refüj %.0f m2, y %.0f..%.0f' % (poly.area, dis0.bounds[1], dis0.bounds[3]))
    # lamps and trees of the north part (refuj-tespit) and of the south part (refuj-guney-tespit)
    pg = yol(A['yollar'].get('refuj_guney_nesneler', ''))
    if pg and os.path.exists(pg):
        ng = json_oku(pg); nesneler = dict(lambalar=nesneler['lambalar'] + ng['lambalar'], agaclar=nesneler['agaclar'] + ng['agaclar'])
    lambalar = []
    for l in nesneler['lambalar']:
        x, y = l['xy']; i = ta.query([x, y])[1]
        if not poly.contains(Point(x, y)):
            continue
        lambalar.append(dict(ad=l['ad'], x=x, y=y, z=zcim(x, y), yol_yonu_deg=float(np.degrees(np.arctan2(tv[i, 1], tv[i, 0]))),
                             boy=Rf['lamba_boy'], kol=Rf['lamba_kol']))
    # trees only on the grass, the trunk (0.3 m) clear of the curb strip ("ic" is the curb's outer edge: a tree there
    # stood on the curb, user 2026-11-15: "kaldırıma denk gelen ağaç varsa kaldır"), not on a lamp
    agaclar = []; cim_ic = unary_union([g_['cim'] for _, g_ in adalar if not g_['cim'].is_empty]).buffer(-0.3)
    ic_ = unary_union([g_['ic'] for _, g_ in adalar]); n_kaldirim = 0
    for t in nesneler['agaclar']:
        x, y = t['xy']
        if not cim_ic.contains(Point(x, y)):
            n_kaldirim += ic_.contains(Point(x, y)); continue
        if any(np.hypot(x - l['x'], y - l['y']) < 1.5 for l in lambalar):
            continue
        h = float(t['boy']); agaclar.append(dict(tur='agac' if h >= 4.5 else 'cali', x=x, y=y, z=zcim(x, y), boy=h,
                                                 tac=round(0.32 * h if h >= 4.5 else 0.45 * h, 2)))
    # meshes of all pieces together
    for _, g_ in adalar[1:]:
        for vk, tk in (('KV', 'KT'), ('CV', 'CT'), ('BV', 'BT'), ('SV', 'ST')):
            o = len(g[vk]); g[vk] = g[vk] + g_[vk]; g[tk] = g[tk] + [i + o for i in g_[tk]]
    X0, X1, Y0g, Y1g = A['nizamiye']['awsim']['izgara'][:4]
    reg = poly.buffer(Rf['bulut_kenar_m']).union(box(X0, Y0g, X1, Y1g)).simplify(0.5)
    alanlar = [dict(xy=[round(float(c), 2) for c in np.array(g.exterior.coords)[:, :2].ravel()]) for g in getattr(reg, 'geoms', [reg])]
    print('  kaldırım %d, çim %d, bordür %d, kenar asfaltı %d üçgen; lamba %d, ağaç %d, çalı %d; bulut alanı %.0f m2'
          % (len(g['KT']) // 3, len(g['CT']) // 3, len(g['BT']) // 3, len(g['ST']) // 3, len(lambalar), sum(t['tur'] == 'agac' for t in agaclar),
             sum(t['tur'] == 'cali' for t in agaclar), reg.area))
    if n_kaldirim:
        print('  kaldırım şeridine denk gelen %d ölçülmüş ağaç/çalı konmadı' % n_kaldirim)
    r3 = lambda V: [round(c, 3) for v in V for c in v]
    return dict(aciklama='Median island y %.0f..%.0f: %.2f m curbs (%.0f cm) on both sides, concrete curb face, grass mound (+%.0f cm); '
                         'two-arm lamps and trees placed from real lidar + ZED camera.' % (dis0.bounds[1], dis0.bounds[3], KG, KH * 100, TUM * 100),
                sinir_xy=[round(float(c), 3) for c in loop[:, :2].ravel()], kaldirim_v=r3(g['KV']), kaldirim_t=g['KT'], cim_v=r3(g['CV']),
                cim_t=g['CT'], bordur_v=r3(g['BV']), bordur_t=g['BT'], asfalt_v=r3(g['SV']), asfalt_t=g['ST'],
                ek_sinirlar=[dict(xy=[round(float(c), 3) for c in lp[:, :2].ravel()]) for lp, _ in adalar[1:]],
                lambalar=lambalar, agaclar=agaclar, bulut_alanlari=alanlar)


# ---------- south part of the main road median (campus junction -> y ~1610) ----------
def guney_kenarlar(A, H):
    """Inner borders of the two carriageways south of the junction at y ~1620:
    east = left border of the northbound lanes (south -> north), west = left border of the southbound lanes (north -> south)."""
    G = A['refuj']['guney']; sol = lambda lid: H.P(H.R[lid][0]['left'])
    east = np.concatenate([sol(l) if i == 0 else sol(l)[1:] for i, l in enumerate(G['dogu_seritler'])])
    west = np.concatenate([sol(l) if i == 0 else sol(l)[1:] for i, l in enumerate(G['bati_seritler'])])
    return east, west


def guney_eksen(A, east, west):
    """Median axis (midpoints of the two inner borders, 1 m steps, south -> north) between y_bas and y_son."""
    from scipy.ndimage import uniform_filter1d
    G = A['refuj']['guney']; e2, w2 = east[:, :2], west[:, :2]; tw = cKDTree(w2); mids = []
    for e in e2:
        d, j = tw.query(e)
        if d < 25:
            mids.append((e + w2[j]) / 2)
    mids = np.array(mids); mids = mids[(mids[:, 1] > G['y_bas'] - 30) & (mids[:, 1] < G['y_son'] + 30)]
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(mids, axis=0).T))]; ss = np.arange(0, s[-1], 1.0)
    ax = np.c_[np.interp(ss, s, mids[:, 0]), np.interp(ss, s, mids[:, 1])]
    ax = np.c_[uniform_filter1d(ax[:, 0], 9, mode='nearest'), uniform_filter1d(ax[:, 1], 9, mode='nearest')]
    ax = ax[(ax[:, 1] >= G['y_bas']) & (ax[:, 1] <= G['y_son'])]
    t = np.gradient(ax, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]
    return ax, t, np.c_[-t[:, 1], t[:, 0]]


def guney_tespit(A):
    """Lamps (two-arm heads 7-11.5 m spanning > 4 m across) and trees (trunks with a crown) in the south median,
    the same method as tespit(); lamp gaps are filled at the regular spacing only BETWEEN two lidar lamps."""
    from scipy.signal import find_peaks
    G = A['refuj']['guney']
    H = Harita(yol(A['yollar']['harita'])); east, west = guney_kenarlar(A, H); ax, tv, nv = guney_eksen(A, east, west)
    S_ = len(ax); tax = cKDTree(ax)

    def sn(P):
        _, i = tax.query(P); v = P - ax[i]; return i + (v * tv[i]).sum(1), (v * nv[i]).sum(1)

    def xy(s_, n):
        i = int(np.clip(round(s_), 0, S_ - 1)); return ax[i] + tv[i] * (s_ - i) + nv[i] * n
    p = pcd_oku(yol(A['yollar']['gercek_lidar']))
    lo, hi = ax.min(0) - 15, ax.max(0) + 15
    p = p[(p[:, 0] > lo[0]) & (p[:, 0] < hi[0]) & (p[:, 1] > lo[1]) & (p[:, 1] < hi[1]), :3]
    qs, qn = sn(p[:, :2]); keep = (abs(qn) < 12) & (qs >= 0) & (qs < S_); q, qs, qn = p[keep], qs[keep], qn[keep]
    print(f'  güney eksen {S_} m (y {ax[0, 1]:.0f}..{ax[-1, 1]:.0f}), lidar noktası {len(q)}')
    Gr = np.full((S_, 48), np.nan); is_ = np.clip(qs.astype(int), 0, S_ - 1); in_ = np.clip(((qn + 12) / 0.5).astype(int), 0, 47)
    key = is_ * 48 + in_; o = np.argsort(key); ks = key[o]; z = q[o, 2]; u, st = np.unique(ks, return_index=True); en = np.r_[st[1:], len(ks)]
    for k, a, b in zip(u, st, en):
        if b - a >= 3:
            Gr.flat[k] = np.percentile(z[a:b], 10)
    Gf = Gr.copy(); Gf[np.isnan(Gf)] = 1e9; Gm = ndimage.minimum_filter(Gf, size=(5, 7)); Gm[Gm > 1e8] = np.nan
    idx = ndimage.distance_transform_edt(np.isnan(Gm), return_distances=False, return_indices=True); Gm = Gm[idx[0], idx[1]]
    hag = q[:, 2] - Gm[is_, in_]
    se, ne = sn(east[:, :2]); sw, nw = sn(west[:, :2])
    o_e = np.argsort(se); o_w = np.argsort(sw)
    he = np.interp(np.arange(S_), se[o_e], -ne[o_e]); hw = np.interp(np.arange(S_), sw[o_w], nw[o_w])
    H_E = np.interp(qs, np.arange(S_), he); H_W = np.interp(qs, np.arange(S_), hw)
    inmed = (qn > -H_E + 0.3) & (qn < H_W - 0.3)
    mh = (hag > 7) & (hag < 11.5) & (qn > -H_E - 1) & (qn < H_W + 1)
    sb = np.arange(0, S_ + 0.5, 0.5); cnt, _ = np.histogram(qs[mh], sb); nspan = np.zeros(len(cnt))
    for i in range(len(cnt)):
        m = mh & (qs >= sb[i]) & (qs < sb[i + 1])
        if m.sum() >= 3:
            nspan[i] = np.percentile(qn[m], 95) - np.percentile(qn[m], 5)
    bg = ndimage.uniform_filter1d(cnt.astype(float), 13)
    score = np.where(nspan > 4.0, cnt / (bg + 1.0), 0)
    # the median trees also reach 7-11 m, so single peaks are not enough: the lamps stand in a row whose spacing is
    # ~lamba_aralik but whose phase drifts along the road. Chain from the strongest peak both ways: at every step look
    # for a peak within 4 m of the expected spot (snap) or fill the spot; stop after 3 missing steps in a row (fills of
    # a stopped chain are dropped). Repeat from the next strong peak that is not near a lamp yet.
    pk, _ = find_peaks(score, height=1.3, distance=30)
    s_ad = np.array([sb[i] + 0.25 for i in pk]); puan = score[pk]; D_ = G['lamba_aralik']; secili = {}
    for i0_ in np.argsort(-puan):
        if puan[i0_] < 3.0:
            break
        if any(abs(s_ad[i0_] - v) < D_ / 2 for v in secili):
            continue
        secili[s_ad[i0_]] = 'lidar (güçlü)'
        for yon in (1, -1):
            cur, kayip, bekleyen = s_ad[i0_], 0, []
            while 0 <= cur + yon * D_ <= S_:
                exp_ = cur + yon * D_; near = [(abs(si - exp_), si, pu) for si, pu in zip(s_ad, puan) if abs(si - exp_) < 4]
                if any(abs(exp_ - v) < D_ / 2 for v in secili if v != cur):
                    break                                            # reached another chain
                if near:
                    _, si, pu = min(near)
                    for b_ in bekleyen: secili[b_] = 'dizi (eşit aralık)'
                    bekleyen = []; secili[si] = 'lidar (güçlü)' if pu >= 3.0 else 'lidar (zayıf, dizide)'; cur = si; kayip = 0
                else:
                    kayip += 1; bekleyen.append(exp_); cur = exp_
                    if kayip >= 3:
                        break
    out = sorted(secili.items())
    # a gap between two chains that holds a whole number of spacings (within 15 %) is filled evenly
    dolu = []
    for (a, sa), (b, sb_) in zip(out[:-1], out[1:]):
        dolu.append((a, sa)); k = int(round((b - a) / D_))
        if k >= 2 and abs((b - a) / k - D_) < 0.15 * D_:
            dolu += [(a + (b - a) * j / k, 'dizi (zincirler arası)') for j in range(1, k)]
    if out:
        dolu.append(out[-1])
    out = dolu
    print('  lamba zinciri: %d lamba (aralık %.1f m)' % (len(out), D_))
    lambalar = [dict(xy=xy(s, 0).tolist(), kaynak=src, ad='GL%02d' % (i + 1)) for i, (s, src) in enumerate(out)]
    ms = (hag > 0.7) & (hag < 2.3) & inmed; r = 0.25
    gi = (qs[ms] / r).astype(int); gj = ((qn[ms] + 12) / r).astype(int)
    Gs = np.zeros((int(S_ / r) + 2, int(24 / r) + 2), int); np.add.at(Gs, (gi, gj), 1)
    lab, _ = ndimage.label(Gs > 0, structure=np.ones((3, 3))); govde = []
    for k, sl in enumerate(ndimage.find_objects(lab), 1):
        sub = (lab[sl] == k); w = Gs[sl] * sub; c_ = w.sum(); ext = max(sub.shape) * r
        if c_ < 5 or ext > 1.8:
            continue
        ii, jj = np.nonzero(sub)
        s0 = (sl[0].start + np.average(ii, weights=w[ii, jj]) + .5) * r; n0 = (sl[1].start + np.average(jj, weights=w[ii, jj]) + .5) * r - 12
        near = (abs(qs - s0) < 0.5) & (abs(qn - n0) < 0.5); top = float(np.percentile(hag[near], 99)) if near.sum() > 3 else 0
        govde.append(dict(s=float(s0), xy=xy(s0, n0).tolist(), nokta=int(c_), tepe=round(top, 1)))
    agaclar = []
    for t in sorted([g for g in govde if g['tepe'] >= 2.5], key=lambda a: -a['nokta']):
        p_ = np.array(t['xy'])
        if any(np.hypot(*(p_ - np.array(l['xy']))) < 1.5 for l in lambalar) or any(np.hypot(*(p_ - np.array(u['xy']))) < 1.5 for u in agaclar):
            continue
        agaclar.append(dict(xy=p_.tolist(), s=t['s'], boy=round(max(t['tepe'], 4.0), 1), buyuk=t['tepe'] >= 6))
    agaclar.sort(key=lambda a: a['s'])
    for i, t in enumerate(agaclar):
        t['ad'] = 'GA%03d' % (i + 1)
    print(f'  güney lamba {len(lambalar)} ({sum(l["kaynak"] == "lidar (güçlü)" for l in lambalar)} güçlü lidar), ağaç {len(agaclar)} (büyük {sum(t["buyuk"] for t in agaclar)})')
    return dict(lambalar=lambalar, agaclar=agaclar)


def guney_tespit_calistir(A, hedef=None):
    print('refuj-guney-tespit')
    veri = guney_tespit(A); hedef = hedef or yol(A['yollar']['refuj_guney_nesneler'])
    json_yaz(hedef, veri, A if hedef == yol(A['yollar']['refuj_guney_nesneler']) else None)
    print('  ->', hedef)
    return veri


def guney_geometri(A, H, nesneler=None):
    """South median island between the inner borders (y_bas .. y_son), corners rounded; curbs, grass, curb faces and the
    asphalt strip as the north median; lamps and trees from veri/refuj_guney_nesneler.json; point-cloud region."""
    from shapely.geometry import Polygon, Point
    Rf = A['refuj']; G = Rf['guney']; KG, KH, TUM = Rf['kaldirim_genislik'], Rf['kaldirim_yukseklik'], Rf['tumsek']
    p = yol(A['yollar'].get('refuj_guney_nesneler', ''))
    if nesneler is None:
        if not p or not os.path.exists(p):
            print('  güney refüj: nesne dosyası yok (duzenle.py refuj-guney-tespit), atlandı'); return None
        nesneler = json_oku(p)
    east, west = guney_kenarlar(A, H); ax, tv, nv = guney_eksen(A, east, west)
    me = (east[:, 1] >= G['y_bas']) & (east[:, 1] <= G['y_son']); mw = (west[:, 1] >= G['y_bas']) & (west[:, 1] <= G['y_son'])
    ring = np.r_[east[me], west[mw]]                      # east south->north, west north->south: a closed ring
    poly = Polygon(ring[:, :2]).buffer(0)
    poly = poly.buffer(-1.5, join_style=1).buffer(1.5, join_style=1)        # rounded noses
    if poly.geom_type != 'Polygon':
        poly = max(poly.geoms, key=lambda g_: g_.area)
    tz = cKDTree(np.r_[east, west][:, :2]); zz = np.r_[east, west][:, 2]
    loop = halka(np.array(poly.exterior.coords), z_fn=lambda x, y: float(zz[tz.query([x, y])[1]]))
    g = kaldirimli_ada(loop, KG, KH, TUM, Rf.get('kenar_payi_m', 0.0))
    zcim = g['zcim']; ta = cKDTree(ax)
    lambalar = []
    for l in nesneler['lambalar']:
        x, y = l['xy']; i = ta.query([x, y])[1]
        lambalar.append(dict(ad=l['ad'], x=x, y=y, z=zcim(x, y), yol_yonu_deg=float(np.degrees(np.arctan2(tv[i, 1], tv[i, 0]))),
                             boy=Rf['lamba_boy'], kol=Rf['lamba_kol']))
    agaclar = []; cim_ic = g['cim'].buffer(-0.3); n_dis = 0
    for t in nesneler['agaclar']:
        x, y = t['xy']
        if not cim_ic.contains(Point(x, y)):
            n_dis += 1; continue
        if any(np.hypot(x - l['x'], y - l['y']) < 1.5 for l in lambalar):
            continue
        h = float(t['boy']); agaclar.append(dict(tur='agac' if h >= 4.5 else 'cali', x=x, y=y, z=zcim(x, y), boy=h,
                                                 tac=round(0.32 * h if h >= 4.5 else 0.45 * h, 2)))
    reg = poly.buffer(Rf['bulut_kenar_m']).simplify(0.5)
    print('  güney refüj %.0f m2, y %.0f..%.0f; kaldırım %d, çim %d, bordür %d, kenar asfaltı %d üçgen; lamba %d, ağaç %d, çalı %d '
          '(çimin dışında kalan %d atlandı); bulut alanı %.0f m2' % (poly.area, loop[:, 1].min(), loop[:, 1].max(), len(g['KT']) // 3, len(g['CT']) // 3,
          len(g['BT']) // 3, len(g['ST']) // 3, len(lambalar), sum(t['tur'] == 'agac' for t in agaclar), sum(t['tur'] == 'cali' for t in agaclar), n_dis, reg.area))
    return dict(loop=loop, g=g, lambalar=lambalar, agaclar=agaclar,
                bulut_alanlari=[dict(xy=[round(float(c), 2) for c in np.array(r_.exterior.coords)[:, :2].ravel()]) for r_ in getattr(reg, 'geoms', [reg])])
