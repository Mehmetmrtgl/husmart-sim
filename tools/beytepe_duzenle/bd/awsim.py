"""Step "awsim": write the AWSIM scene data (Assets/Beytepe/Data/duzenlemeler.json).

Two sections, both read by Assets/Beytepe/Editor/BeytepeDuzenleme.cs while the scene is built:
  nizamiye : terrain regrade grids (target height + weight), grass / marking-removal / road masks,
             asphalt mesh of the new lanes + the drawn asphalt areas, lane markings, bus shelter pose
  refuj    : median island (curbs, grass mound, curb faces, lamps, trees) and the point-cloud sampling regions
Coordinates are in the ROS map frame (x, y, z). Grids are row by row (y), x fastest.
"""
import os, base64
import numpy as np
from matplotlib.path import Path
from scipy import ndimage
from .ortak import Harita, yol, json_oku, json_yaz
from . import cizim, refuj


def _seritler(H):
    """lanelet id -> (left way, right way, tags) for lanelets only."""
    return {lid: (m['left'], m['right'], t) for lid, (m, t) in H.R.items() if t.get('type') == 'lanelet' and 'left' in m and 'right' in m}


def _nesneler(A):
    """veri/nizamiye_nesneler.json (empty dict if missing)"""
    p = yol(A['yollar'].get('nizamiye_nesneler', ''))
    return json_oku(p) if p and os.path.exists(p) else {}


def nizamiye_verisi(A, H, Ho, plan, D):
    Z, C = A['nizamiye'], A['cizim']; AW = Z['awsim']
    Rl, Rlo = _seritler(H), _seritler(Ho)
    R = plan['R']; yeni = set(R.values()); cut = plan['cut']
    pts = lambda HH, wid: HH.P(wid)
    poly = lambda k: np.array(D[k][1])
    Aref = np.array([v for v in H.N.values() if -230 < v[0] < -80 and 2480 < v[1] < 2800])

    def zr(x, y):     # ground height = inverse-distance mean of the 8 nearest lanelet nodes
        d = np.hypot(Aref[:, 0] - x, Aref[:, 1] - y); k = np.argpartition(d, 8)[:8]; w = 1 / (d[k] + 0.3) ** 2
        return float((Aref[k, 2] * w).sum() / w.sum())

    X0, X1, Y0, Y1, RES = AW['izgara']
    nx, ny = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
    gx = X0 + (np.arange(nx) + .5) * RES; gy = Y0 + (np.arange(ny) + .5) * RES
    GX, GY = np.meshgrid(gx, gy); GP = np.c_[GX.ravel(), GY.ravel()]
    rast = lambda p: Path(np.asarray(p)[:, :2]).contains_points(GP).reshape(ny, nx)
    serit_alani = lambda HH, l, r: np.r_[pts(HH, l)[:, :2], pts(HH, r)[::-1, :2]]
    disk = lambda m: (np.hypot(*np.mgrid[-int(m / RES):int(m / RES) + 1, -int(m / RES):int(m / RES) + 1]) * RES <= m)
    dil = lambda a, m: ndimage.binary_dilation(a, disk(m)); ero = lambda a, m: ndimage.binary_erosion(a, disk(m))
    icerde = lambda P_, e=0: ((P_[:, 0] > X0 - e) & (P_[:, 0] < X1 + e) & (P_[:, 1] > Y0 - e) & (P_[:, 1] < Y1 + e)).any()

    # ---- masks on the grid
    LANEU = np.zeros((ny, nx), bool)                      # new lanes
    for lid in yeni:
        l, r, _ = Rl[lid]; LANEU |= rast(serit_alani(H, l, r))
    OLDRM = np.zeros((ny, nx), bool)                      # removed part of the old southbound lanes
    for lid in AW['eski_seritler']:
        l, r, _ = Rlo[lid]; Lp = pts(Ho, l)[:cut[l] + 1]; Rp = pts(Ho, r)[:cut[r] + 1]; OLDRM |= rast(np.r_[Lp[:, :2], Rp[::-1, :2]])
    KEPT = np.zeros((ny, nx), bool)                       # existing lanes that stay
    for lid, (l, r, t) in Rl.items():
        if lid in yeni or t.get('subtype', 'road') != 'road':
            continue
        P_ = serit_alani(H, l, r)
        if icerde(P_):
            KEPT |= rast(P_)
    # ---- road and islands as exact polygons: new lanes + existing lanes + drawn asphalt; gaps narrower than
    #      2 x bosluk_doldur_m become asphalt, areas enclosed by asphalt (and the drawn islands) become raised islands
    from shapely.geometry import Polygon as SP, box as SB
    from shapely.ops import unary_union
    LU = unary_union([SP(serit_alani(H, *Rl[lid][:2])).buffer(0) for lid in yeni])
    KU = unary_union([SP(serit_alani(H, l, r)).buffer(0) for lid, (l, r, t) in Rl.items()
                      if lid not in yeni and t.get('subtype', 'road') == 'road' and icerde(serit_alani(H, l, r))])
    ASF = unary_union([SP(poly(k)).buffer(0) for k in C['asfalt']])
    bd_ = AW['bosluk_doldur_m']
    YOL = unary_union([LU, KU, ASF]).buffer(bd_, join_style=2).buffer(-bd_, join_style=2).intersection(SB(X0, Y0, X1, Y1))
    # islands = ONLY what the user drew as island (cizim.ada) + guard booths (a hole of the road with a booth in it,
    # or the booth + 0.8 m). Every other undrawn gap between the roads (holes, pockets, open strips next to islands)
    # becomes asphalt: where curbs go is decided by the drawing, not guessed here.
    kulube = [SP(poly(k)).buffer(0) for k in C.get('kulube', []) if k in D]
    kulubeli = lambda a_: any(a_.intersects(k_) for k_ in kulube)
    tek = lambda g_: max(getattr(g_, 'geoms', [g_]), key=lambda h: h.area)
    delik = [SP(h) for g_ in getattr(YOL, 'geoms', [YOL]) for h in g_.interiors if SP(h).area >= 1.0]
    adalar = [SP(poly(k)).buffer(0) for k in C['ada']] + [h for h in delik if kulubeli(h)]
    YOL = unary_union([YOL] + [h for h in delik if not kulubeli(h)]).buffer(0)
    for kp in kulube:
        if not any(a_.contains(kp.centroid) for a_ in adalar):
            y_ = kp.buffer(0.8, join_style=2).buffer(-0.5).buffer(0.5).difference(unary_union(adalar)).buffer(0)
            if not y_.is_empty:
                adalar.append(tek(y_))
    adalar = [tek(a_) for a_ in adalar]   # touching islands stay separate (the booth pad must not take the grass island)
    ADA = unary_union(adalar).buffer(0)
    # pockets and open areas up to 2 x ada_dolgu_m wide next to an island -> asphalt (no bare ground at road level);
    # areas that do not touch an island (roadside) are left as they are
    df = AW['ada_dolgu_m']; YA = unary_union([YOL, ADA]).buffer(0)
    EK = YA.buffer(df, join_style=2).buffer(-df, join_style=2).intersection(SB(X0, Y0, X1, Y1)).difference(YA)
    TUM = YA.buffer(bd_, join_style=2).buffer(-bd_, join_style=2)
    cepler = [SP(h).buffer(0) for g_ in getattr(TUM, 'geoms', [TUM]) for h in g_.interiors]
    for e in list(getattr(EK, 'geoms', [EK])) + cepler:
        if e.geom_type == 'Polygon' and e.area >= 0.5 and any(a_.distance(e) < 0.3 for a_ in adalar):
            YOL = unary_union([YOL, e]).buffer(0)
    YOL = unary_union([YOL, TUM.difference(ADA)]).buffer(0)     # slivers between islands and road
    rk = AW['ada_kose_yaricap_m']
    ADA0 = unary_union(adalar).buffer(0)
    adalar = [tek(a_.buffer(-rk).buffer(rk)) for a_ in adalar]
    adalar = [a_ for a_ in adalar if a_.area >= 5.0]
    ADA = unary_union(adalar).buffer(0)
    YOL = unary_union([YOL, ADA0]).difference(ADA).buffer(0)
    # ---- retaining walls ("onunde_kaldirim"): the gap between the road and the wall face becomes a paved sidewalk at road
    #      level (a fully paved island with a curb); added last, so the pocket filling above never sees it
    from shapely.geometry import LineString as SL, Point as SPt
    NS = _nesneler(A); duvarlar = [w for w in NS.get('duvarlar', []) if w.get('onunde_kaldirim')]; kaldirimlar = []
    # objects standing on the sidewalk (kiosk, flagpoles, banner poles): the sidewalk also covers their footprint
    # (in front of the red kiosk the real curb is ~0.8 m east of the drawn asphalt edge), but stays 0.5 m off the lanes
    ayak = [SP([(k['x'] + sx * 0.5 * k.get('derin', 1.2) * np.cos(np.radians(k.get('yuz_deg', 0))) - sy * 0.5 * k.get('en', 1.6) * np.sin(np.radians(k.get('yuz_deg', 0))),
                 k['y'] + sx * 0.5 * k.get('derin', 1.2) * np.sin(np.radians(k.get('yuz_deg', 0))) + sy * 0.5 * k.get('en', 1.6) * np.cos(np.radians(k.get('yuz_deg', 0))))
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]).buffer(0.4, join_style=2) for k in NS.get('kulubeler', [])]
    ayak += [SPt(o['x'], o['y']).buffer(0.5) for k in ('bayraklar', 'panolar') for o in NS.get(k, [])]
    SERIT = unary_union([LU, KU]).buffer(0.5)
    for w in duvarlar:
        Lw = SL(np.array(w['xy']).reshape(-1, 2))
        bant = Lw.buffer(-8.0, single_sided=True, cap_style=2)       # right of the drawing direction = road side
        k_ = bant.difference(unary_union([YOL, ADA]).buffer(0))
        k_ = unary_union([k_] + [a_.intersection(bant) for a_ in ayak]).difference(SERIT).difference(ADA)
        k_ = k_.buffer(0.3, join_style=2).buffer(-0.35, join_style=2).buffer(0.05).intersection(bant).difference(SERIT).buffer(0)
        parca = [g_ for g_ in getattr(k_, 'geoms', [k_]) if g_.geom_type == 'Polygon']
        if parca:
            k_ = max(parca, key=lambda g_: g_.intersection(Lw.buffer(0.3)).area)   # the piece along the wall
            if k_.area >= 5.0:
                kaldirimlar.append(k_)
                print('  duvar önü kaldırım %s: %.0f m2, en %.1f..%.1f m' % (w['ad'], k_.area,
                      min(Lw.interpolate(s).distance(YOL) for s in np.linspace(0.5, Lw.length - 0.5, 40)),
                      max(Lw.interpolate(s).distance(YOL) for s in np.linspace(0.5, Lw.length - 0.5, 40))))
    adalar += kaldirimlar; ADA = unary_union(adalar).buffer(0)
    if kaldirimlar:
        YOL = YOL.difference(unary_union(kaldirimlar)).buffer(0)

    def rast_sh(geom):
        m = np.zeros((ny, nx), bool)
        for g_ in getattr(geom, 'geoms', [geom]):
            if g_.is_empty or g_.geom_type != 'Polygon':
                continue
            m |= rast(np.array(g_.exterior.coords))
            for h in g_.interiors:
                m &= ~rast(np.array(h.coords))
        return m
    ISL = rast_sh(ADA)
    F = (LANEU | rast_sh(YOL.difference(KU.buffer(0.3)))) & ~ISL
    Cm = ((dil(OLDRM, 1.5) & ~F) | ISL) & ~dil(KEPT, 0.3)  # becomes grass
    M = (dil(OLDRM, 1.5) | dil(F, 0.3) | ISL) & ~ero(KEPT, 0.5)   # old RoadRunner markings removed here
    # RoadRunner road pieces of the old loop left next to the new road (jagged 1 m edges) turn into grass; the shoulders
    # of the existing roads are kept (material change only, no regrade)
    Cm_cikti = Cm | (dil(F, AW['eski_yol_cim_m']) & ~F & ~dil(KEPT, AW['mevcut_yol_koruma_m']))
    # no trees on asphalt, existing lanes or island curbs
    # islands with a guard booth on them are fully paved (no grass), the others get a curb ring + grass
    KG = A['refuj']['kaldirim_genislik']
    tam_beton = [kulubeli(a_) for a_ in adalar[:len(adalar) - len(kaldirimlar)]] + [True] * len(kaldirimlar)
    seritler = [a_ if tb else a_.difference(a_.buffer(-KG, join_style=2)) for a_, tb in zip(adalar, tam_beton)]
    AGS = (F | dil(KEPT, 0.5) | rast_sh(unary_union(seritler))) if adalar else F | dil(KEPT, 0.5)
    R0 = F | Cm
    Zg = np.full((ny, nx), np.nan)
    for a, b in zip(*np.where(R0)):
        Zg[a, b] = zr(gx[b], gy[a]) - 0.03
    # behind a retaining wall the ground is at the wall top (band 1.0..2.5 m behind the face, 10 cm under the top); the slope
    # starts from there. The first metre behind the face stays at road level: the terrain triangles (<= 1 m) climb from low to
    # high behind the wall, hidden by it, instead of poking up through the sidewalk at the wall foot
    ARKA = np.zeros((ny, nx), bool)
    for w in duvarlar:
        P_ = np.array(w['xy']).reshape(-1, 2); hs = np.array(w['boy'], float); Lw = SL(P_)
        s_ = np.r_[0, np.cumsum(np.hypot(*np.diff(P_, axis=0).T))]
        dip = rast_sh(Lw.buffer(1.0, single_sided=True, cap_style=2)) & ~R0
        for a, b in zip(*np.where(dip)):
            Zg[a, b] = zr(gx[b], gy[a]) - 0.03
        ARKA |= dip
        arka = rast_sh(Lw.buffer(2.5, single_sided=True, cap_style=2)) & ~R0 & ~dip
        for a, b in zip(*np.where(arka)):
            s = Lw.project(SPt(gx[b], gy[a])); q = Lw.interpolate(s)
            Zg[a, b] = zr(q.x, q.y) + float(np.interp(s, s_, hs)) + 0.1
        ARKA |= arka
    R0b = R0 | ARKA
    dist, ind = ndimage.distance_transform_edt(~R0b, sampling=RES, return_indices=True)
    T = Zg[ind[0], ind[1]]; t = np.clip(1 - dist / AW['sev_m'], 0, 1); Wt = t * t * (3 - 2 * t); Wt[R0b] = 1.0
    # existing lanes (+ gutter / curb margin) keep their RoadRunner shape: the slope regrade must not warp them
    # (it used to, under speed bump 2000446, so the flat bump sank into the road at its corners); 1.5 m soft edge
    # speed bumps are rigid flat objects: the ground under them (+0.5 m) is never regraded, even inside the new road
    kasis = [P_ for P_ in H.kasisler.values() if icerde(P_, 5)]
    KASIS = np.zeros((ny, nx), bool)
    for P_ in kasis:
        KASIS |= rast(P_)
    KASIS = dil(KASIS, 0.5)
    koru = (dil(KEPT, AW['mevcut_yol_kenar_m']) & ~R0) | KASIS
    Wt = Wt * (1 - ndimage.uniform_filter(koru.astype(float), size=3)); Wt[R0b & ~KASIS] = 1.0
    print('  asfalt %d m2, çim %d m2 (ada %d), çizgi silme %d m2' % (F.sum() * RES * RES, Cm.sum() * RES * RES, ISL.sum() * RES * RES, M.sum() * RES * RES))

    # ---- where the new lanes join the existing ones, every overlay (asphalt, markings) starts just under the old road
    #      and rises to its normal height over baglanti_rampa_m: no step at the joint
    baglanti = [np.array([H.N[n][:2] for n in Z['kuzey_baslangic_dugumleri']]),
                np.array([H.N[H.W[w][0]][:2] for w in Z['guney_yollari']])]

    def alcalma(x, y):
        p = np.array([x, y]); d = 1e9
        for P in baglanti:
            for a, b in zip(P[:-1], P[1:]):
                ab = b - a; t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1); d = min(d, np.hypot(*(a + t * ab - p)))
        u = np.clip(d / AW['baglanti_rampa_m'], 0, 1)
        return AW['baglanti_alcalma_m'] * (1 - u * u * (3 - 2 * u))

    # ---- asphalt mesh: one strip per new lane
    V, Tr = [], []

    def res(a, n):
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(a[:, :2], axis=0), axis=1))]; s /= s[-1]; u = np.linspace(0, 1, n)
        return np.c_[[np.interp(u, s, a[:, k]) for k in range(3)]].T
    strips = []
    for k, lid in R.items():
        l, r, _ = Rl[lid]; Lp = pts(H, l); Rp = pts(H, r); strips.append(serit_alani(H, l, r))
        n = max(len(Lp), len(Rp)); Lr = res(Lp, n); Rr = res(Rp, n); b = len(V); dz = 0.025 if k in ('U', 'E3', 'CEP') else 0.02
        for p in Lr:
            V.append([p[0], p[1], p[2] + dz - alcalma(p[0], p[1])])
        for p in Rr:
            V.append([p[0], p[1], p[2] + dz - alcalma(p[0], p[1])])
        for j in range(n - 1):
            a0, a1, c0, c1 = b + j, b + j + 1, b + n + j, b + n + j + 1; Tr.extend([a0, c0, a1, a1, c0, c1])
    nstrip = len(Tr) // 3

    # ---- the rest of the asphalt (drawn areas + filled gaps, minus islands): exact polygon edges, cut into 1 m pieces
    G = YOL.difference(ADA).difference(KU.buffer(0.3)).difference(LU.buffer(-0.15)).buffer(0)
    if kasis:   # no extra asphalt over a speed bump (it would cover the bump's end)
        G = G.difference(unary_union([SP(P_[:, :2]).buffer(0.3) for P_ in kasis])).buffer(0)
    npc = refuj.ucgenle(G, V, Tr, lambda x, y: zr(x, y) + 0.015 - alcalma(x, y))
    print('  ek asfalt %d m2 (%d üçgen)' % (round(G.area), npc))

    # ---- lane markings of the new lanes; skipped where a border runs inside another lane (forks / merges).
    #      Colours as on the real road: solid lines (edges, curb side) yellow, dashed lane lines white.
    CZ = {False: ([], []), True: ([], [])}          # dashed? -> (vertices, triangles)

    def cizgi_ciz(p, dashed, w=0.15):
        MV, MT = CZ[dashed]
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]; L = s[-1]; ss = np.r_[np.arange(0, L, 0.5), L]
        q = np.c_[[np.interp(ss, s, p[:, k]) for k in range(3)]].T; on = ((ss % 8.0) < 3.0) if dashed else np.ones(len(ss), bool)
        d = np.gradient(q[:, :2], axis=0); d /= np.linalg.norm(d, axis=1)[:, None] + 1e-9; n = np.c_[d[:, 1], -d[:, 0]]
        for k in range(len(q) - 1):
            if not (on[k] and (on[k + 1] or not dashed)):
                continue
            b = len(MV)
            for qq, nn in ((q[k], n[k]), (q[k + 1], n[k + 1])):
                zz = qq[2] + 0.035 - alcalma(qq[0], qq[1])
                MV.append([qq[0] - nn[0] * w / 2, qq[1] - nn[1] * w / 2, zz]); MV.append([qq[0] + nn[0] * w / 2, qq[1] + nn[1] * w / 2, zz])
            MT.extend([b, b + 2, b + 1, b + 1, b + 2, b + 3])
    lane_polys = []
    for lid, (l, r, t) in Rl.items():
        if t.get('subtype', 'road') != 'road':
            continue
        P_ = serit_alani(H, l, r)
        if icerde(P_, 20):
            lane_polys.append(({l, r}, Path(P_), P_))

    def kenar_uzaklik(p, Pp):
        a = Pp[:-1]; b = Pp[1:]; ab = b - a; t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
        return np.min(np.hypot(*(a + t[:, None] * ab - p).T))
    # The U-turn's outer edge line is not painted on its lane border but along the guardhouse island's curb, as on the real
    # road (ZED frame 11925: the yellow line wraps round the island, the turning area itself has no line). Since the
    # U-turn lane follows the real drive path, its outer border runs up to ~1.7 m in front of the island and a line there
    # floated in the open asphalt. The line: 0.25 m in front of the island edge, where that edge is within 2.5 m of the border.
    ur = SL(pts(H, plan['w']['Ur'])[:, :2]); ada_yolu = ADA.buffer(0.25).boundary.intersection(ur.buffer(2.5))
    for g_ in getattr(ada_yolu, 'geoms', [ada_yolu]):
        if g_.geom_type != 'LineString' or g_.length < 1.0:
            continue
        c_ = np.array(g_.coords); c_ = np.c_[c_, [zr(x_, y_) for x_, y_ in c_]]
        cizgi_ciz(c_, False)
        print('  U dönüşü dış çizgisi ada kenarında: %.1f m' % g_.length)
    nskip = 0
    for k, wid in plan['w'].items():
        if k == 'Ur':
            continue
        p = pts(H, wid); s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]; ss = np.r_[np.arange(0, s[-1], 0.5), s[-1]]
        q = np.c_[[np.interp(ss, s, p[:, kk]) for kk in range(3)]].T; keep = np.ones(len(q), bool)
        if k != 'Ul':       # the turning area is open (no lines), except the median-side line and the island line above
            for kk in range(len(q)):
                P_ = SPt(q[kk, 0], q[kk, 1])
                if ur.distance(P_) < 2.5 and ADA.distance(P_) > 0.8:
                    keep[kk] = False
        for ws, pa, Pp in lane_polys:
            if wid in ws:
                continue
            for kk in np.where(pa.contains_points(q[:, :2]))[0]:
                if kenar_uzaklik(q[kk, :2], Pp) > 0.3:
                    keep[kk] = False
        nskip += (~keep).sum(); i = 0
        while i < len(q):
            if not keep[i]:
                i += 1; continue
            j = i
            while j + 1 < len(q) and keep[j + 1]:
                j += 1
            if j > i:
                cizgi_ciz(q[i:j + 1], H.WT[wid].get('subtype') == 'dashed')
            i = j + 1

    # ---- bus shelter at the drawn pin, facing along the bay lane (north)
    dk = np.array(D[C['durak']][1])[0]; yb = float(dk[1]); xd = float(dk[0])
    cc = np.array(plan['cep_orta']); k0 = np.argmin(abs(cc[:, 1] - yb))
    dd = cc[max(k0 - 3, 0)] - cc[min(k0 + 3, len(cc) - 1)]; yaw = float(np.degrees(np.arctan2(dd[1], dd[0])))
    durak = dict(x=xd, y=yb, z=zr(xd, yb) + 0.02, yaw_deg=yaw)
    print('  asfalt üçgen %d (şerit %d), çizgi köşe %d beyaz + %d sarı, çatalda atlanan çizgi noktası %d, durak (%.1f, %.1f) yön %.0f°'
          % (len(Tr) // 3, nstrip, len(CZ[True][0]), len(CZ[False][0]), nskip, xd, yb, yaw))
    b64f = lambda a: base64.b64encode(np.nan_to_num(a, nan=-999).astype(np.float32).ravel().tobytes()).decode()
    b64u = lambda a: base64.b64encode(a.astype(np.uint8).ravel().tobytes()).decode()
    veri = dict(aciklama="Gate road (nizamiye): drawn routes -> lanelet lanes; asphalt, markings, terrain regrade, bus shelter.",
                x0=X0, y0=Y0, cozunurluk=RES, nx=nx, ny=ny, zemin_hedef=b64f(T), zemin_agirlik=b64f(Wt), cim_maske=b64u(Cm),
                cizgi_sil_maske=b64u(M), yol_maske=b64u(F),
                asfalt_v=[round(c, 3) for v in V for c in v], asfalt_t=Tr, cizgi_v=[round(c, 3) for v in CZ[True][0] for c in v], cizgi_t=CZ[True][1],
                cizgi_sari_v=[round(c, 3) for v in CZ[False][0] for c in v], cizgi_sari_t=CZ[False][1],
                durak=durak, bulut_kutu=[X0, X1, Y0, Y1], bulut_yogunluk=A['bulut']['yogunluk'])
    # ---- raised islands (curb + grass) on the asphalt edge, like the median
    Rf = A['refuj']; ada_veri = []
    for a_ in adalar:
        loop = refuj.halka(np.array(a_.exterior.coords), z_fn=lambda x, y: zr(x, y) + 0.015)
        kg = 1e3 if tam_beton[len(ada_veri)] else Rf['kaldirim_genislik']
        ada_veri.append((loop, refuj.kaldirimli_ada(loop, kg, Rf['kaldirim_yukseklik'], AW['ada_tumsek'])))
    print('  kaldırımlı ada: ' + ', '.join('%.0f m2 (%.0f, %.0f)%s' % (g_['dis'].area, g_['dis'].centroid.x, g_['dis'].centroid.y, ' tam beton' if tb else '')
                                          for (_, g_), tb in zip(ada_veri, tam_beton)))
    veri['cim_maske'] = b64u(Cm_cikti); veri['agacsiz_maske'] = b64u(AGS)
    sekil = dict(asfalt=unary_union([G, LU]).buffer(0), mevcut=KU, adalar=ADA)
    return veri, dict(F=F, C=Cm, M=M, T=T, W=Wt, gx=gx, gy=gy), ada_veri, sekil


def kaplama_kontrol(A, H, sekil, ring):
    """Between the outermost lane lines of the gate area, every bit of ground must be asphalt (new or existing lane),
    an island or the median. Prints the bare spots (grass at road level) and writes the shapes for the KML step."""
    from shapely.geometry import Polygon as SP, box as SB, mapping
    from shapely.ops import unary_union
    X0, X1, Y0, Y1 = A['nizamiye']['awsim']['izgara'][:4]; ust = A['nizamiye']['kuzey_uc_y']
    kapali = unary_union([sekil['asfalt'], sekil['mevcut'].buffer(0.3), sekil['adalar'], SP(ring).buffer(0)]).buffer(0.05)
    cizgiler = [H.P(w)[:, :2] for lid, (m, t) in H.R.items() if t.get('type') == 'lanelet' and t.get('subtype', 'road') == 'road' and 'left' in m
                for w in (m['left'], m['right'])]
    bos = []
    for y in np.arange(Y0 + 1, min(Y1, ust) - 0.5, 1.0):
        xs = [P_[:, 0][abs(P_[:, 1] - y) < 0.8] for P_ in cizgiler]
        xs = np.concatenate([x for x in xs if len(x)]) if any(len(x) for x in xs) else np.array([])
        xs = xs[(xs > X0) & (xs < X1)]
        if len(xs) < 2:
            continue
        acik = SB(xs.min(), y - 0.25, xs.max(), y + 0.25).difference(kapali)
        bos += [g for g in getattr(acik, 'geoms', [acik]) if g.area > 0.25]
    if bos:
        parca = unary_union([b.buffer(0.3) for b in bos])
        for g in getattr(parca, 'geoms', [parca]):
            print('  UYARI: iki yol arasında kaplanmamış zemin (yol hizasında çim): x %.0f..%.0f, y %.0f..%.0f'
                  % (g.bounds[0], g.bounds[2], g.bounds[1], g.bounds[3]))
    else:
        print('  kaplama kontrolü: nizamiyede iki yol arasında açıkta zemin yok')
    poly_xy = lambda g: [np.round(np.array(p.exterior.coords), 2).tolist() for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon']
    json_yaz(os.path.join(yol(A['yollar']['ara']), 'nizamiye_yuzey.json'),
             dict(asfalt=poly_xy(sekil['asfalt']), adalar=poly_xy(sekil['adalar']), bos=[np.round(np.array(b.exterior.coords), 2).tolist() for b in bos]))
    return bos


def cizgi_renk_verisi(A, H, ring):
    """RoadRunner draws the tour roads' lines the other way round (yellow centre / dashes, white edges). Every painted line of
    a lanelet on the tour route (real drive) gets the colour seen in the real ZED frames, from its role in the map:
      shared by two lanelets of the same direction (lane divider)        -> white
      shared by two opposite lanelets (centre line of a two-way road)    -> white (yellow on the west campus road: sari_orta_kutu)
      only one lanelet, its right side (outer edge)                      -> yellow (white on the southbound main road, except sari_dis_kenar_y)
      only one lanelet, its left side (median-side edge)                 -> yellow
    Every rule is decided per segment (a lanelet line can be over 1 km long and curve away from the median).
    Output: region (box around the lines) + the lines as segments; the scene builder recolours each RoadRunner marking
    triangle after the lanelet line it lies on (0.6 m, parallel); markings off these lines (crossings, arrows) stay.
    Two more passes:
      kenar_*_xy: the edge lines again, turned so that the road is on their left (ax ay bx by). On the campus roads RoadRunner
                  has no painted edge line but a light concrete gutter strip (~0.5 m) along the curb, where the real road has
                  asphalt and a yellow edge line: gutter triangles running along an edge become asphalt, their road-side
                  oluk_cizgi_m the edge colour. The virtual outer borders of junction lanelets count as edges here (only
                  here): where a curb runs past a side road, RoadRunner has a gutter along them too
      alan:       rings of the tour lanelet area (even-odd; without the gate and sari_orta_kutu): RoadRunner's yellow centre
                  lines through junctions, where the lanelet borders are virtual, become white, if they run along the road
                  (nearest lanelet line within 5 m, virtual borders sanal_xy included; speed-bump stripes run across it)"""
    import collections
    from shapely.geometry import Polygon as SP, Point, box as SBox
    from shapely.ops import unary_union
    from scipy.spatial import cKDTree
    from .kenar import gercek_iz
    Z = A['cizgiler']; tiz = cKDTree(gercek_iz(A, 2.0)); REF = SP(ring).buffer(0)
    rol = collections.defaultdict(list); tur = set(); poly = {}
    for lid, (m, t) in H.R.items():
        if t.get('type') != 'lanelet' or t.get('subtype', 'road') != 'road' or 'left' not in m:
            continue
        L, R = H.P(m['left'])[:, :2], H.P(m['right'])[:, :2]
        C = (L[np.linspace(0, len(L) - 1, 10).astype(int)] + R[np.linspace(0, len(R) - 1, 10).astype(int)]) / 2
        if (tiz.query(C)[0] < Z['rota_m']).mean() >= Z['rota_oran']:
            tur.add(lid)
        if np.dot(L[-1] - L[0], R[-1] - R[0]) < 0:
            R = R[::-1]
        poly[lid] = SP(np.r_[L, R[::-1]]).buffer(0)
        rol[m['left']].append((lid, 'left')); rol[m['right']].append((lid, 'right'))
    kx0, kx1, ky0, ky1 = Z['sari_orta_kutu']; sy0, sy1 = Z['sari_dis_kenar_y']
    kutu = lambda p: kx0 < p[0] < kx1 and ky0 < p[1] < ky1
    sari, beyaz, ksari, kbeyaz, sanal, say = [], [], [], [], [], collections.Counter()
    r3 = lambda *v: [round(float(x), 3) for x in v]
    for w, l in rol.items():
        if not any(x[0] in tur for x in l):
            continue
        virt = H.WT[w].get('type') == 'virtual'
        if virt:      # junction lanelet borders: the road direction for the third pass; an outer one (one lanelet) may still
            P = H.P(w)[:, :2]   # have a RoadRunner gutter along it (curb past a side road), so it is an edge for the gutter pass
            for a, b in zip(P[:-1], P[1:]):
                sanal.extend(r3(a[0], a[1], b[0], b[1]))
            if len(l) != 1:
                continue
        elif H.WT[w].get('type') not in ('line_thin', 'line_thick', 'road_border'):
            continue
        P = H.P(w)[:, :2]; roles = sorted(r for _, r in l)
        if len(P) < 2:
            continue
        if roles == ['left', 'right']:
            k = 'ayirici'
        elif roles == ['left', 'left']:
            k = 'orta'
        elif len(l) == 1:
            k = 'dis_kenar' if l[0][1] == 'right' else 'ic_kenar'; lid = l[0][0]
        else:
            continue
        for a, b in zip(P[:-1], P[1:]):
            mid = (a + b) / 2; d = b - a; n = np.hypot(*d)
            if n < 1e-3:
                continue
            if k == 'ayirici':
                s_ = False
            elif k == 'orta':
                s_ = kutu(mid)
            else:
                # turn the segment so that the road is on its left; for the right border that is the driving direction
                if not poly[lid].contains(Point(mid + 0.4 * np.array([-d[1], d[0]]) / n)):
                    a, b, d = b, a, -d
                if k == 'dis_kenar':
                    s_ = not (d[1] < 0 and REF.distance(Point(mid)) < Z['beyaz_dis_kenar_refuj_m'] and not (sy0 < mid[1] < sy1))
                else:
                    s_ = True
                (ksari if s_ else kbeyaz).extend(r3(a[0], a[1], b[0], b[1]))
            say[(k + ('_sanal' if virt else ''), 'sarı' if s_ else 'beyaz')] += 1
            if not virt:      # nothing is painted on a virtual border
                (sari if s_ else beyaz).extend(r3(a[0], a[1], b[0], b[1]))
    # tour lanelet area, without the gate (own lanes / markings) and the west campus road (yellow centre)
    gx0, gx1, gy0, gy1 = A['nizamiye']['awsim']['izgara'][:4]
    ALAN = unary_union([poly[i] for i in tur]).buffer(0.05).buffer(-0.05).difference(SBox(gx0, gy0, gx1, gy1)).difference(SBox(kx0, ky0, kx1, ky1))
    alan = [dict(xy=[round(float(v), 2) for q in np.array(r.coords)[:-1] for v in q])
            for p in getattr(ALAN, 'geoms', [ALAN]) if p.area > 1 for r in [p.simplify(0.1).exterior, *p.simplify(0.1).interiors]]
    S_ = np.array(sari + beyaz).reshape(-1, 2); x0, y0 = S_.min(0) - 3; x1, y1 = S_.max(0) + 3
    print('  çizgi renkleri: tur üzerindeki %d şerit; parça: %s; oluk geçişi %d sarı / %d beyaz kenar parçası; tur alanı %d halka %.0f m²' % (
        len(tur), ', '.join('%s %s %d' % (k, r, n) for (k, r), n in sorted(say.items())), len(ksari) // 4, len(kbeyaz) // 4, len(alan), ALAN.area))
    return dict(bolge_xy=[round(float(v), 2) for v in (x0, y0, x1, y0, x1, y1, x0, y1)], sari_xy=sari, beyaz_xy=beyaz,
                kenar_sari_xy=ksari, kenar_beyaz_xy=kbeyaz, sanal_xy=sanal, alan=alan, oluk_cizgi_m=float(Z['oluk_cizgi_m']))


def nesne_verisi(A, H):
    """Gate objects (barriers, flags, kiosk, sign, generator, wall, guardrail, fence) from veri/nizamiye_nesneler.json.
    They are measured by hand (real lidar + ZED), so here they are only checked against the lanes:
    nothing may stand on a lane, except barrier posts, which stand on the line between two lanes."""
    p = yol(A['yollar'].get('nizamiye_nesneler', ''))
    if not p or not os.path.exists(p):
        print('  nesne dosyası yok, nizamiye nesneleri atlandı'); return None
    from shapely.geometry import Polygon, Point, LineString
    from shapely.ops import unary_union
    v = json_oku(p)
    tum = [o for k in ('bariyerler', 'bayraklar', 'kulubeler', 'tabelalar', 'kutular', 'panolar', 'cop_kutulari', 'yuvarlak_calilar', 'sus_agaclari') for o in v.get(k, [])]
    xy = [(o['x'], o['y']) for o in tum] + [c for k in ('duvarlar', 'korkuluklar', 'citler') for o in v.get(k, [])
                                            for c in zip(o['xy'][0::2], o['xy'][1::2])]
    lo, hi = np.min(xy, 0) - 20, np.max(xy, 0) + 20
    serit = []
    for lid, (mm, t) in H.R.items():
        if t.get('type') != 'lanelet' or 'left' not in mm:
            continue
        L, R = H.P(mm['left'])[:, :2], H.P(mm['right'])[:, :2]
        P = np.r_[L, R[::-1]]
        if ((P > lo).all(1) & (P < hi).all(1)).any():
            g = Polygon(P).buffer(0)
            if g.area > 0:
                serit.append(g)
    YOL = unary_union(serit); CIZGI = unary_union([g.boundary for g in serit])   # every lane border, also the shared ones
    sorun = []
    for o in v.get('bariyerler', []):
        d = Point(o['x'], o['y']).distance(CIZGI)
        print('  bariyer %s: en yakın şerit çizgisine %.2f m' % (o['ad'], d))
        if YOL.contains(Point(o['x'], o['y'])) and d > 0.5:
            sorun.append('bariyer %s şeridin %.2f m içinde (çizgiden uzak)' % (o['ad'], d))
    for k in ('bayraklar', 'kulubeler', 'tabelalar', 'kutular', 'panolar', 'cop_kutulari', 'yuvarlak_calilar', 'sus_agaclari'):
        for o in v.get(k, []):
            if k == 'panolar':     # the banner hangs from the pole towards yon_deg
                a = np.radians(o.get('yon_deg', 0)); w_ = o.get('en', 1.0) + 0.12
                if LineString([(o['x'], o['y']), (o['x'] + w_ * np.cos(a), o['y'] + w_ * np.sin(a))]).buffer(0.06).intersects(YOL):
                    sorun.append('pano %s şeridin üstüne uzanıyor' % o['ad'])
                continue
            r = 0.5 * max(o.get('en', 0.2), o.get('derin', 0.2))
            if Point(o['x'], o['y']).buffer(r).intersects(YOL):
                sorun.append('%s %s şeridin üstünde' % (k, o['ad']))
    for o in v.get('buyuk_kulubeler', []):      # body and roof outline in the booth frame -> ROS
        t_ = np.radians(o['aci_deg']); U_ = np.array([np.cos(t_), np.sin(t_)]); V_ = np.array([-np.sin(t_), np.cos(t_)]); C_ = np.array([o['x'], o['y']])
        def cerceve(r, vs, va):
            uv = [(r, vs)] + [(r * np.cos(a_), va + r * np.sin(a_)) for a_ in np.linspace(0, np.pi, 25)] + [(-r, vs)]
            return Polygon([tuple(C_ + U_ * u + V_ * w) for u, w in uv])
        govde = cerceve(o['yari_en'], o['guney_v'], o['yay_v']); cati = cerceve(o['yari_en'] + o['tasma'], o['guney_v'] - o['tasma_guney'], o['yay_v'])
        if govde.buffer(0.5).intersects(YOL):
            sorun.append('büyük kulübe %s gövdesi şeritlere 0.5 m den yakın' % o['ad'])
        print('  büyük kulübe %s: gövde şeritlere %.2f m, çatı kenarı %.2f m (çatı %s)' % (o['ad'], govde.distance(YOL), cati.distance(YOL),
              'şeridin üstüne taşıyor' if cati.intersects(YOL) else 'şeritlerin dışında'))
    for k in ('duvarlar', 'korkuluklar', 'citler'):
        for o in v.get(k, []):
            ln = LineString(list(zip(o['xy'][0::2], o['xy'][1::2])))
            if ln.intersects(YOL):
                sorun.append('%s %s şeritleri kesiyor' % (k, o['ad']))
            else:
                print('  %s %s: %.0f m, şeritlere en az %.2f m' % (k, o['ad'], ln.length, ln.distance(YOL)))
    print('  nesneler: %d bariyer, %d bayrak, %d kulübe, %d tabela, %d kutu, %d pano, %d çöp kutusu, %d çalı, %d süs ağacı, %d duvar, %d korkuluk (%.0f m), %d çit' % (
        *[len(v.get(k, [])) for k in ('bariyerler', 'bayraklar', 'kulubeler', 'tabelalar', 'kutular', 'panolar', 'cop_kutulari', 'yuvarlak_calilar', 'sus_agaclari', 'duvarlar', 'korkuluklar')],
        sum(LineString(np.array(o['xy']).reshape(-1, 2)).length for o in v.get('korkuluklar', [])), len(v.get('citler', []))))
    if sorun:
        raise SystemExit('HATA: nizamiye nesneleri:\n  ' + '\n  '.join(sorun))
    return {k: v[k] for k in ('bariyerler', 'bayraklar', 'kulubeler', 'buyuk_kulubeler', 'tabelalar', 'kutular', 'panolar', 'cop_kutulari', 'yuvarlak_calilar', 'sus_agaclari', 'duvarlar', 'korkuluklar', 'citler') if k in v}


def calistir(A, hedef=None):
    H = Harita(yol(A['yollar']['harita'])); Ho = Harita(yol(A['yollar']['temel_harita']))
    ara = yol(A['yollar']['ara'])
    plan = json_oku(os.path.join(ara, 'nizamiye_plan.json')); D = cizim.oku(A)
    print('awsim: nizamiye')
    niz, izgara, ada_veri, sekil = nizamiye_verisi(A, H, Ho, plan, D)
    np.savez(os.path.join(ara, 'nizamiye_izgara.npz'), **izgara)
    print('awsim: refüj')
    ref = refuj.geometri(A, H, plan)
    from . import kenar
    YOL = kenar.yol_alani(H); TK = A['yol_kenari']
    ref['agaclar'], n_k, n_a = kenar.tac_kirp(ref['agaclar'], YOL, TK['tac_yol_payi_m'], TK['tac_en_az_m'])
    print('  refüj ağaçları: %d tacı şeritten %.1f m geride kalacak şekilde küçültüldü, %d çok dar kaldığı için konmadı'
          % (n_k, TK['tac_yol_payi_m'], n_a))
    for loop, g_ in ada_veri:     # gate islands share the median meshes (same curb / grass look)
        for vk, tk, Vs, Ts in [('kaldirim_v', 'kaldirim_t', g_['KV'], g_['KT']), ('cim_v', 'cim_t', g_['CV'], g_['CT']), ('bordur_v', 'bordur_t', g_['BV'], g_['BT'])]:
            o = len(ref[vk]) // 3
            ref[vk] += [round(c, 3) for v in Vs for c in v]; ref[tk] += [i + o for i in Ts]
        ref['ek_sinirlar'].append(dict(xy=[round(float(c), 3) for c in loop[:, :2].ravel()]))
    # (the south part of the median, y 300 .. 1706, is part of the median island itself: refuj.geometri)
    json_yaz(os.path.join(ara, 'bulut_alanlari.json'), ref['bulut_alanlari'])        # for "bulut"
    json_yaz(os.path.join(ara, 'refuj_ozet.json'), {k: ref[k] for k in ('sinir_xy', 'lambalar', 'agaclar')})   # for "kml" / "kontrol"
    json_yaz(os.path.join(ara, 'durak.json'), niz['durak'])
    kaplama_kontrol(A, H, sekil, np.array(ref['sinir_xy']).reshape(-1, 2))
    renk = cizgi_renk_verisi(A, H, np.array(ref['sinir_xy']).reshape(-1, 2))
    print('awsim: nizamiye nesneleri')
    nes = nesne_verisi(A, H)
    print('awsim: korkuluklar')
    from . import korkuluk
    kk = korkuluk.geometri(A, H)            # guardrails + concrete parapets along the tour route (veri/korkuluklar.json)
    if kk:
        nes = nes or {}
        nes['korkuluklar'] = nes.get('korkuluklar', []) + kk['korkuluklar']
        nes['duvarlar'] = nes.get('duvarlar', []) + kk['duvarlar']
    veri = dict(aciklama='Beytepe scene edits generated by tools/beytepe_duzenle (husmart-sim). ROS map frame.', nizamiye=niz, refuj=ref,
                cizgi_renk=renk)
    if nes:
        veri['nesneler'] = nes
        # the point-cloud map is regenerated from AWSIM around every object too (3 m), so the map matches the simulated lidar
        # also where an object stands outside the median region (e.g. the guardrails along the main road)
        from shapely.geometry import LineString, Point
        from shapely.ops import unary_union
        parca = [LineString(np.array(o['xy']).reshape(-1, 2)).buffer(3.0) for k in ('duvarlar', 'korkuluklar', 'citler') for o in nes.get(k, [])]
        parca += [Point(o['x'], o['y']).buffer(3.0) for k in ('bariyerler', 'bayraklar', 'kulubeler', 'buyuk_kulubeler', 'tabelalar', 'kutular', 'panolar',
                                                            'cop_kutulari', 'yuvarlak_calilar', 'sus_agaclari') for o in nes.get(k, [])]
        ek = unary_union(parca).simplify(0.3)
        n0 = len(ref['bulut_alanlari'])
        for g_ in getattr(ek, 'geoms', [ek]):
            ref['bulut_alanlari'].append(dict(xy=[round(float(c), 2) for q in np.array(g_.exterior.coords)[:-1] for c in q]))
        json_yaz(os.path.join(ara, 'bulut_alanlari.json'), ref['bulut_alanlari'])
        print('  bulut bölgesine nesne çevresi eklendi: %d çokgen, %.0f m2' % (len(ref['bulut_alanlari']) - n0, ek.area))
    print('awsim: yol kenarı ağaçları')
    from shapely.geometry import Polygon, box
    from shapely.ops import unary_union
    adalar = unary_union([Polygon(np.array(ref['sinir_xy']).reshape(-1, 2)).buffer(0)] +
                         [Polygon(np.array(e['xy']).reshape(-1, 2)).buffer(0) for e in ref['ek_sinirlar']])
    kutu = box(niz['x0'], niz['y0'], niz['x0'] + niz['nx'] * niz['cozunurluk'], niz['y0'] + niz['ny'] * niz['cozunurluk'])
    ken = kenar.geometri(A, H, adalar, kutu)
    if ken:
        ken['agaclar'], n_k, n_a = kenar.tac_kirp(ken['agaclar'], YOL, TK['tac_yol_payi_m'], TK['tac_en_az_m'])
        print('  yol kenarı: %d tacı şeritten %.1f m geride kalacak şekilde küçültüldü, %d çok dar kaldığı için konmadı'
              % (n_k, TK['tac_yol_payi_m'], n_a))
        veri['yol_kenari'] = ken
        ref['bulut_alanlari'] += ken['gizle']     # trees replaced -> the map is regenerated from AWSIM in the whole corridor
        json_yaz(os.path.join(ara, 'bulut_alanlari.json'), ref['bulut_alanlari'])
    print('awsim: yol yüzeyi düzeltme')
    from . import yol_duzelt
    yd, yd_alan = yol_duzelt.veri(A)       # RoadRunner road steps / creases the real road does not have (ayarlar.yaml yol_duzelt)
    if yd:
        veri['yol_duzelt'] = yd
        ref['bulut_alanlari'] += yd_alan     # the map is regenerated from AWSIM where the road surface changes
        json_yaz(os.path.join(ara, 'bulut_alanlari.json'), ref['bulut_alanlari'])
    hedef = hedef or yol(A['yollar']['awsim_veri'])
    json_yaz(hedef, veri, A if hedef == yol(A['yollar']['awsim_veri']) else None)
    print('awsim: %.1f MB -> %s' % (os.path.getsize(hedef) / 1e6, hedef))
    return veri
