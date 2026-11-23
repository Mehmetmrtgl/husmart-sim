"""Step "lanelet": build the gate (nizamiye) lanes from the drawn vehicle routes and write the lanelet map.

Input : the base map (yollar.temel_harita) + the drawing (veri/cizim_xy.json, step "cizim").
Output: yollar.harita (the base map with the new lanes, edited as text so the diff stays small)
        veri/nizamiye_plan.json (lanelet / way ids of the new lanes, used by the "awsim" step).

The drawn routes are lane CENTRES. A route pair (left + right) gives three borders: left route - w/2,
the middle of the two routes, right route + w/2. Lane ids are allocated after the largest id in the base map,
in a fixed order, so running the step again gives the same ids.
"""
import os, re
import xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import CubicSpline
from .ortak import Harita, Izdusum, yol, json_yaz, yedekle
from . import cizim


# ---------- geometry helpers ----------
def xfun(pts):
    """x as a function of y along a north-south route (natural cubic spline, clamped at the ends)."""
    p = np.array(pts); p = p[np.argsort(p[:, 1])]
    _, u = np.unique(p[:, 1].round(3), return_index=True); p = p[u]
    if len(p) >= 4:
        cs = CubicSpline(p[:, 1], p[:, 0], bc_type='natural')
        return lambda y: cs(np.clip(y, p[0, 1], p[-1, 1]))
    return lambda y: np.interp(y, p[:, 1], p[:, 0])


def orta(f, g):
    return lambda y: (f(y) + g(y)) / 2


def ys(a, b, step=1.0):
    return np.linspace(a, b, max(2, int(abs(b - a) / step) + 1))


def kaydir(c, off):
    """Offset a polyline (in travel order); off > 0 = right of travel."""
    d = np.gradient(c, axis=0); d /= np.linalg.norm(d, axis=1)[:, None]
    return c + off * np.c_[d[:, 1], -d[:, 0]]


def cizgi(fx, y0, y1, off):
    y = ys(y0, y1)
    return kaydir(np.c_[fx(y), y], off)


def harmanla(pts, a=None, b=None, L=20.0):
    """Pull the first / last point onto a and b; the shift fades out over L metres."""
    pts = np.array(pts, float); s = np.r_[0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
    if a is not None:
        pts += np.clip(1 - s / L, 0, 1)[:, None] * (np.asarray(a) - pts[0])
    if b is not None:
        pts += np.clip(1 - (s[-1] - s) / L, 0, 1)[:, None] * (np.asarray(b) - pts[-1])
    return pts


def hermite(p0, t0, p1, t1, n):
    L = np.linalg.norm(p1 - p0); t0 = t0 / np.linalg.norm(t0) * L; t1 = t1 / np.linalg.norm(t1) * L
    u = np.linspace(0, 1, n)[:, None]
    return (2*u**3 - 3*u**2 + 1)*p0 + (u**3 - 2*u**2 + u)*t0 + (-2*u**3 + 3*u**2)*p1 + (u**3 - u**2)*t1


def segment_y(a, b, y):
    t = (y - a[1]) / (b[1] - a[1]); return a + t * (b - a)


def poly_uzaklik(p, poly):
    best = 1e9
    for a, b in zip(poly[:-1], poly[1:]):
        t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1); best = min(best, np.hypot(*(a + t * (b - a) - p)))
    return best


# ---------- OSM text writers ----------
def node_xml(i, x, y, z, I):
    lat, lon = I.xy2ll(x, y)
    return (f'  <node id="{i}" lat="{lat:.11f}" lon="{lon:.11f}">\n    <tag k="local_x" v="{x:.4f}" />\n'
            f'    <tag k="local_y" v="{y:.4f}" />\n    <tag k="ele" v="{z:.4f}" />\n  </node>\n')


def way_xml(i, nodes, sub):
    return (f'  <way id="{i}">\n' + ''.join(f'    <nd ref="{k}" />\n' for k in nodes)
            + f'    <tag k="type" v="line_thin" />\n    <tag k="subtype" v="{sub}" />\n  </way>\n')


def rel_xml(i, l, r, ad, hiz, notu):
    return (f'  <relation id="{i}">\n    <member type="way" ref="{l}" role="left" />\n    <member type="way" ref="{r}" role="right" />\n'
            '    <tag k="type" v="lanelet" />\n    <tag k="subtype" v="road" />\n    <tag k="location" v="urban" />\n'
            '    <tag k="one_way" v="yes" />\n    <tag k="participant:vehicle" v="yes" />\n'
            f'    <tag k="speed_limit" v="{hiz}" />\n    <tag k="note" v="{notu}: {ad}" />\n  </relation>\n')


def set_way(metin, wid, nodes):
    pat = re.compile(r'(  <way id="%s">\n)((?:    <nd ref="\d+" />\n)+)' % wid)
    assert len(pat.findall(metin)) == 1, wid
    return pat.sub(lambda m: m.group(1) + ''.join(f'    <nd ref="{k}" />\n' for k in nodes), metin)


class Uretici:
    """Allocates new node / way / lanelet ids in order and remembers what to write."""

    def __init__(self, H):
        self.H = H; self.nid = max(H.ids) + 1
        self.yeni_dugum, self.yeni_yol, self.yeni_serit = {}, {}, {}
        # ele of a new node = inverse-distance mean of the 6 nearest base-map nodes around the gate
        self.REF = np.array([v for v in H.N.values() if -220 < v[0] < -100 and 2480 < v[1] < 2700])

    def yeni_id(self):
        self.nid += 1; return str(self.nid - 1)

    def z(self, x, y):
        d = np.hypot(self.REF[:, 0] - x, self.REF[:, 1] - y); k = np.argpartition(d, 6)[:6]; w = 1 / (d[k] + 0.5) ** 2
        return float((self.REF[k, 2] * w).sum() / w.sum())

    def dugum(self, p):
        i = self.yeni_id(); z = self.z(p[0], p[1])
        self.H.N[i] = np.array([p[0], p[1], z]); self.yeni_dugum[i] = (p[0], p[1], z); return i

    def sinir(self, pts, ilk=None, son=None, L=20.0):
        """Border line: optionally starts / ends at existing nodes (the geometry is blended onto them over L m)."""
        P = lambda i: self.H.N[i][:2]
        pts = harmanla(pts, P(ilk) if ilk else None, P(son) if son else None, L)
        return [ilk or self.dugum(pts[0])] + [self.dugum(p) for p in pts[1:-1]] + [son or self.dugum(pts[-1])]

    def sinir_komsu(self, pts, ilk, son, komsu, L=20.0):
        """Like sinir(), but wherever the line would lie right of (outside) the neighbouring border `komsu`
        (node ids, same travel direction) it runs ON that border: its nodes are shared. Only the parts before and
        after (e.g. the fork / merge tapers of a bay) get new nodes; the switch points lie on `komsu`."""
        P = lambda i: self.H.N[i][:2]
        pts = harmanla(pts, P(ilk), P(son), L); K = np.array([P(i) for i in komsu])
        sK = np.r_[0, np.cumsum(np.hypot(*np.diff(K, axis=0).T))]

        def izdus(p):          # signed distance to komsu (< 0: right of travel) and arc length of the foot point
            best = (1e9, 0.0, 0.0)
            for j, (a, b) in enumerate(zip(K[:-1], K[1:])):
                d = b - a; t = np.clip(np.dot(p - a, d) / np.dot(d, d), 0, 1); q = a + t * d; m = np.hypot(*(p - q))
                if m < abs(best[0]):
                    best = (m if d[0] * (p - a)[1] - d[1] * (p - a)[0] >= 0 else -m, sK[j] + t * np.hypot(*d), 0.0)
            return best[:2]
        sd = np.array([izdus(p)[0] for p in pts]); dis = np.flatnonzero(sd < -0.05)
        if len(dis) == 0:
            return self.sinir(pts, ilk, son, L=1e-6)
        i1, i2 = dis[0], dis[-1]
        if i1 == 0 or i2 == len(pts) - 1 or len(dis) != i2 - i1 + 1:
            raise SystemExit('HATA: sinir_komsu: çizgi komşu sınırın dışına tek parça halinde ve uçlardan uzakta geçmiyor')
        kes = lambda a, b: pts[a] + sd[a] / (sd[a] - sd[b]) * (pts[b] - pts[a])     # point where the line crosses komsu
        c1, c2 = kes(i1 - 1, i1), kes(i2 + 1, i2); s1, s2 = izdus(c1)[1], izdus(c2)[1]
        ortak = [komsu[j] for j in range(len(komsu)) if s1 + 0.5 < sK[j] < s2 - 0.5]
        return ([ilk] + [self.dugum(p) for p in pts[1:i1]] + [self.dugum(c1)] + ortak + [self.dugum(c2)]
                + [self.dugum(p) for p in pts[i2 + 1:-1]] + [son])

    def yol(self, nodes, sub):
        i = self.yeni_id(); self.yeni_yol[i] = (nodes, sub); self.H.W[i] = nodes; return i

    def serit(self, l, r, ad):
        i = self.yeni_id(); self.yeni_serit[i] = (l, r, ad); return i


def calistir(A, hedef=None):
    Z, C = A['nizamiye'], A['cizim']
    H = Harita(yol(A['yollar']['temel_harita'])); I = Izdusum(H)
    D = cizim.oku(A); rota = lambda k: np.array(D[C['rota'][k]][1])
    U_ = Uretici(H); P = lambda i: H.N[i][:2]
    h = Z['serit_genislik'] / 2

    # ---- routes as x(y) curves
    cL, cR, cC = xfun(rota('cikis_sol')), xfun(rota('cikis_sag')), xfun(rota('cikis_cep'))
    gL = xfun(rota('giris_sol')[:-1])        # the last point of this route is a stray click in the drawing
    gR = xfun(rota('giris_sag'))
    U = rota('donus')
    Y_N0, YB, YA, YTOP = Z['kuzey_baglanti_y'], Z['sag_catal_y'], float(U[0, 1]), Z['kuzey_uc_y']
    YS, YM, YP, YEND = Z['cep_catal_y'], float(U[-1, 1]), Z['cep_birlesme_y'], Z['guney_baglanti_y']
    n0, n1, n2 = Z['kuzey_baslangic_dugumleri']

    # ---- northbound (exit): a = up to the right fork, b = up to the U-turn, c = to the map edge
    w, R = {}, {}
    NB0a = U_.sinir(cizgi(cL, Y_N0, YB, -h), n0); NB1a = U_.sinir(cizgi(orta(cL, cR), Y_N0, YB, 0), n1); NB2a = U_.sinir(cizgi(cR, Y_N0, YB, h), n2)
    NB0b = U_.sinir(cizgi(cL, YB, YA, -h), NB0a[-1]); NB1b = U_.sinir(cizgi(orta(cL, cR), YB, YA, 0), NB1a[-1]); NB2b = U_.sinir(cizgi(cR, YB, YA, h), NB2a[-1])
    NB0c = U_.sinir(cizgi(cL, YA, YTOP, -h), NB0b[-1]); NB1c = U_.sinir(cizgi(orta(cL, cR), YA, YTOP, 0), NB1b[-1]); NB2c = U_.sinir(cizgi(cR, YA, YTOP, h), NB2b[-1])
    E3l = U_.sinir(cizgi(cC, YB, YTOP, -h), NB1a[-1], None, L=1.0); E3r = U_.sinir(cizgi(cC, YB, YTOP, h), NB2a[-1], None, L=1.0)
    for k, v, s in [('NB0a', NB0a, 'solid'), ('NB1a', NB1a, 'dashed'), ('NB2a', NB2a, 'solid'), ('NB0b', NB0b, 'solid'), ('NB1b', NB1b, 'dashed'),
                    ('NB2b', NB2b, 'solid'), ('NB0c', NB0c, 'solid'), ('NB1c', NB1c, 'dashed'), ('NB2c', NB2c, 'solid'), ('E3l', E3l, 'solid'), ('E3r', E3r, 'solid')]:
        w[k] = U_.yol(v, s)
    R['A1'] = U_.serit(w['NB0a'], w['NB1a'], 'kuzey sol'); R['B1'] = U_.serit(w['NB1a'], w['NB2a'], 'kuzey sağ')
    R['A2'] = U_.serit(w['NB0b'], w['NB1b'], 'kuzey sol (dönüş çatalı)'); R['E2a'] = U_.serit(w['NB1b'], w['NB2b'], 'çıkış sağ')
    R['E1'] = U_.serit(w['NB0c'], w['NB1c'], 'çıkış sol'); R['E2b'] = U_.serit(w['NB1c'], w['NB2c'], 'çıkış sağ (devam)')
    R['E3'] = U_.serit(w['E3l'], w['E3r'], 'çıkış cep')

    # ---- southbound (entry): the existing southbound borders are cut at YEND, the new lanes end on the cut nodes
    GY = Z['guney_yollari']

    def kesim(wid, yv):
        p = np.array([P(i) for i in H.W[wid]]); k = int(p[:, 1].argmax()); return k + int(np.argmin(abs(p[k:, 1] - yv)))
    cut = {wid: kesim(wid, YEND) for wid in GY}; en = {wid: H.W[wid][cut[wid]] for wid in GY}
    SB0a = U_.sinir(cizgi(gL, YTOP, YS, -h)); SB1a = U_.sinir(cizgi(orta(gL, gR), YTOP, YS, 0)); SB2a = U_.sinir(cizgi(gR, YTOP, YS, h))
    SB0b = U_.sinir(cizgi(gL, YS, YM, -h), SB0a[-1]); SB1b = U_.sinir(cizgi(orta(gL, gR), YS, YM, 0), SB1a[-1]); SB2b = U_.sinir(cizgi(gR, YS, YM, h), SB2a[-1])
    SB0c = U_.sinir(cizgi(gL, YM, YP, -h), SB0b[-1]); SB1c = U_.sinir(cizgi(orta(gL, gR), YM, YP, 0), SB1b[-1]); SB2c = U_.sinir(cizgi(gR, YM, YP, h), SB2b[-1])
    SB0d = U_.sinir(cizgi(gL, YP, YEND, -h), SB0c[-1], en[GY[0]], L=12.0)
    SB1d = U_.sinir(cizgi(orta(gL, gR), YP, YEND, 0), SB1c[-1], en[GY[1]], L=12.0)
    SB2d = U_.sinir(cizgi(gR, YP, YEND, h), SB2c[-1], en[GY[2]], L=12.0)

    # ---- bus-stop bay: the drawn route (polyline) is kept; only the entry curve is moved cep_giris_uzatma_m
    #      past the drawn corner, the exit corner is rounded, the merge point stays.
    cp = rota('giris_cep'); cp = cp[np.argsort(-cp[:, 1])]          # north -> south (travel order)
    gdir = np.array([gR(YS - 1) - gR(YS), -1.0])
    qj = segment_y(cp[1], cp[2], cp[1][1] - Z['cep_giris_uzatma_m'])
    giris = hermite(cp[0], gdir, qj, cp[2] - cp[1], int(np.hypot(*(qj - cp[0]))) + 2)
    qx = segment_y(cp[2], cp[3], cp[3][1] + 3.0)
    ara = np.r_[[qj], [cp[2]], [qx]]
    aras = np.concatenate([np.linspace(ara[i], ara[i + 1], max(2, int(np.hypot(*(ara[i + 1] - ara[i]))) + 1))[:-1] for i in range(len(ara) - 1)])
    cikis = hermite(qx, cp[3] - cp[2], cp[4], np.array([gR(YP - 1) - gR(YP), -1.0]), int(np.hypot(*(cp[4] - qx))) + 2)
    cc = np.r_[giris[:-1], aras, cikis]
    dev = np.array([poly_uzaklik(p, cp) for p in cc]); yk = cp[1][1] - Z['cep_giris_uzatma_m']
    print('  bay centre vs drawn route: max %.2f m (y %.1f); after the entry curve (y<%.0f) max %.2f m'
          % (dev.max(), cc[dev.argmax(), 1], yk, dev[cc[:, 1] < yk].max()))
    # The bay is a wide area, not one lane: the drawn "giriş sağ cep" lies right next to "giriş sağ şerit". Its left
    # border therefore runs ON the right border of the right entry lane (shared nodes) instead of 3.5 m left of the bay
    # route, which left a ~2.3 m gap; only the fork / merge tapers to the lane divider are its own. The right border
    # stays on the drawn bay route (user: "giriş sağ cepin sağ şerit çizgisinin konumunu bozma").
    Pl = U_.sinir_komsu(kaydir(cc, -h), SB1a[-1], SB1c[-1], SB2b + SB2c[1:], L=4.0)
    Pr = U_.sinir(kaydir(cc, h), SB2a[-1], SB2c[-1], L=4.0)
    for k, v, s in [('SB0a', SB0a, 'solid'), ('SB1a', SB1a, 'dashed'), ('SB2a', SB2a, 'solid'), ('SB0b', SB0b, 'solid'), ('SB1b', SB1b, 'dashed'),
                    ('SB2b', SB2b, 'solid'), ('SB0c', SB0c, 'solid'), ('SB1c', SB1c, 'dashed'), ('SB2c', SB2c, 'solid'), ('SB0d', SB0d, 'solid'),
                    ('SB1d', SB1d, 'dashed'), ('SB2d', SB2d, 'solid'), ('Pl', Pl, 'solid'), ('Pr', Pr, 'solid')]:
        w[k] = U_.yol(v, s)
    R['G1a'] = U_.serit(w['SB0a'], w['SB1a'], 'giriş sol'); R['G2a'] = U_.serit(w['SB1a'], w['SB2a'], 'giriş sağ')
    R['G1b'] = U_.serit(w['SB0b'], w['SB1b'], 'giriş sol (2)'); R['G2b'] = U_.serit(w['SB1b'], w['SB2b'], 'giriş sağ (2)')
    R['G1c'] = U_.serit(w['SB0c'], w['SB1c'], 'giriş sol (dönüş sonrası)'); R['G2c'] = U_.serit(w['SB1c'], w['SB2c'], 'giriş sağ (3)')
    R['G1d'] = U_.serit(w['SB0d'], w['SB1d'], 'giriş sol (bağlantı)'); R['G2d'] = U_.serit(w['SB1d'], w['SB2d'], 'giriş sağ (bağlantı)')
    R['CEP'] = U_.serit(w['Pl'], w['Pr'], 'durak cebi')

    # ---- U-turn (exit -> entry): spline through the drawn route, leaves due north, arrives due south
    u = U.copy(); s = np.r_[0, np.cumsum(np.hypot(*np.diff(u, axis=0).T))]
    cs = CubicSpline(s, u, bc_type=((1, np.array([0, 1.0])), (1, np.array([0, -1.0]))))
    ss = np.linspace(0, s[-1], int(s[-1] / 0.7) + 1); c = cs(ss); d = cs(ss, 1); d /= np.linalg.norm(d, axis=1)[:, None]; nn = np.c_[d[:, 1], -d[:, 0]]
    Ul = U_.sinir(c - h * nn, NB0b[-1], SB0b[-1], L=6.0); Ur = U_.sinir(c + h * nn, NB1b[-1], SB1b[-1], L=6.0)
    w['Ul'] = U_.yol(Ul, 'solid'); w['Ur'] = U_.yol(Ur, 'solid'); R['U'] = U_.serit(w['Ul'], w['Ur'], 'U dönüşü (ring dönüş)')

    # ---- edit the base map text: drop the old northern part of the southbound borders and its orphans, append new elements
    eski = {wid: list(H.W[wid]) for wid in GY}; silinen = set()
    for wid in GY:
        silinen |= set(eski[wid][:cut[wid]])
    silinen -= {n0, n1, n2}
    out = H.metin
    for i in silinen:
        out, k = re.subn(r'  <node id="%s" [^>]*>\n(?:    <tag [^\n]*\n)*  </node>\n' % i, '', out); assert k == 1, i
    for wid in GY:
        out = set_way(out, wid, eski[wid][cut[wid]:])
    for wid in Z['kirpilacak_yollar']:
        nodes = H.W[wid]; son = max(j for j, k in enumerate(nodes) if k in silinen or k in (n0, n1, n2))
        out = set_way(out, wid, nodes[son + 1:])
    pos = out.rfind('  </node>\n') + len('  </node>\n')
    out = out[:pos] + ''.join(node_xml(i, *v, I) for i, v in U_.yeni_dugum.items()) + out[pos:]
    pos = out.rfind('  </way>\n') + len('  </way>\n')
    out = out[:pos] + ''.join(way_xml(i, *v) for i, v in U_.yeni_yol.items()) + out[pos:]
    pos = out.rfind('</osm>')
    out = out[:pos] + ''.join(rel_xml(i, l, r, ad, Z['hiz_siniri'], Z['not']) for i, (l, r, ad) in U_.yeni_serit.items()) + out[pos:]

    # ---- sanity check: every referenced node / way exists
    r2 = ET.fromstring(out.encode()); NN = {n.get('id') for n in r2.iter('node')}; WW = {x.get('id') for x in r2.iter('way')}
    eks_d = sum(1 for x in r2.iter('way') for nd in x.iter('nd') if nd.get('ref') not in NN)
    eks_y = sum(1 for x in r2.iter('relation') for m in x.iter('member') if m.get('type') == 'way' and m.get('ref') not in WW)
    if eks_d or eks_y:
        raise SystemExit(f'HATA: haritada eksik referans (düğüm {eks_d}, yol {eks_y}); dosya yazılmadı')

    hedef = hedef or yol(A['yollar']['harita'])
    if hedef == yol(A['yollar']['harita']):
        yedekle(hedef, A)
    with open(hedef, 'w') as f:
        f.write(out)
    plan = {'R': R, 'w': w, 'Y': dict(N0=Y_N0, B=YB, A=YA, TOP=YTOP, S=YS, M=YM, P=YP, END=YEND), 'cut': cut, 'cep_orta': cc.tolist(),
            'silinen_dugum': len(silinen)}
    json_yaz(os.path.join(yol(A['yollar']['ara']), 'nizamiye_plan.json'), plan)
    print(f'lanelet: {len(U_.yeni_serit)} yeni şerit, {len(U_.yeni_dugum)} yeni düğüm, {len(silinen)} düğüm silindi -> {hedef}')
    return plan
